// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <mutex>
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <cstring>

#include "cameras/l515.h"

struct short3
{
    uint16_t x, y, z;
};

void draw_axes()
{
    glLineWidth(2);
    glBegin(GL_LINES);
    // Draw x, y, z axes
    glColor3f(1, 0, 0); glVertex3f(0, 0, 0);  glVertex3f(-1, 0, 0);
    glColor3f(0, 1, 0); glVertex3f(0, 0, 0);  glVertex3f(0, -1, 0);
    glColor3f(0, 0, 1); glVertex3f(0, 0, 0);  glVertex3f(0, 0, 1);
    glEnd();

    glLineWidth(1);
}

void draw_floor()
{
    glBegin(GL_LINES);
    glColor4f(0.4f, 0.4f, 0.4f, 1.f);
    // Render "floor" grid
    for (int i = 0; i <= 8; i++)
    {
        glVertex3i(i - 4, 1, 0);
        glVertex3i(i - 4, 1, 8);
        glVertex3i(-4, 1, i);
        glVertex3i(4, 1, i);
    }
    glEnd();
}

void render_scene(glfw_state app_state)
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glColor3f(1.0, 1.0, 1.0);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 4.0 / 3.0, 1, 40);

    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();
    gluLookAt(1, 0, 5, 1, 0, 0, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
    glRotated(app_state.pitch, -1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    draw_floor();
}

class camera_renderer
{
public:
    // Takes the calculated angle as input and rotates the 3D camera model accordignly
    void render_camera(float3 theta)
    {

        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);

        glPushMatrix();
        // Set the rotation, converting theta to degrees
        glRotatef(theta.x * 180 / PI, 0, 0, -1);
        glRotatef(theta.y * 180 / PI, 0, -1, 0);
        glRotatef((theta.z - PI / 2) * 180 / PI, -1, 0, 0);

        draw_axes();

        // Scale camera drawing
        glScalef(0.035, 0.035, 0.035);


        glPopMatrix();

        glDisable(GL_BLEND);
        glFlush();
    }

};

class rotation_estimator
{
    // theta is the angle of camera rotation in x, y and z components
    float3 theta;
    std::mutex theta_mtx;
    /* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
    values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
    float alpha = 0.98;
    bool firstGyro = true;
    bool firstAccel = true;
    // Keeps the arrival time of previous gyro frame
    double last_ts_gyro = 0;
public:
    // Function to calculate the change in angle of motion based on data from gyro
    void process_gyro(rs2_vector gyro_data, double ts)
    {
        if (firstGyro) // On the first iteration, use only data from accelerometer to set the camera's initial position
        {
            firstGyro = false;
            last_ts_gyro = ts;
            return;
        }
        // Holds the change in angle, as calculated from gyro
        float3 gyro_angle;

        // Initialize gyro_angle with data from gyro
        gyro_angle.x = gyro_data.x; // Pitch
        gyro_angle.y = gyro_data.y; // Yaw
        gyro_angle.z = gyro_data.z; // Roll

        // Compute the difference between arrival times of previous and current gyro frames
        double dt_gyro = ts - last_ts_gyro;
        last_ts_gyro = ts;

        // Change in angle equals gyro measures * time passed since last measurement
        gyro_angle = gyro_angle * dt_gyro;

        // Apply the calculated change of angle to the current angle (theta)
        std::lock_guard<std::mutex> lock(theta_mtx);
        theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
    }

    void process_accel(rs2_vector accel_data)
    {
        // Holds the angle as calculated from accelerometer data
        float3 accel_angle;

        // Calculate rotation angle from accelerometer data
        accel_angle.z = atan2(accel_data.y, accel_data.z);
        accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

        // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
        std::lock_guard<std::mutex> lock(theta_mtx);
        if (firstAccel)
        {
            firstAccel = false;
            theta = accel_angle;
            // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
            theta.y = PI;
        }
        else
        {
            /*
            Apply Complementary Filter:
                - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                  that are steady over time, is used to cancel out drift.
                - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations
            */
            theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
            theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
        }
    }

    // Returns the current rotation angle
    float3 get_theta()
    {
        std::lock_guard<std::mutex> lock(theta_mtx);
        return theta;
    }
};


bool check_imu_is_supported()
{
    bool found_gyro = false;
    bool found_accel = false;
    rs2::context ctx;
    for (auto dev : ctx.query_devices())
    {
        // The same device should support gyro and accel
        found_gyro = false;
        found_accel = false;
        for (auto sensor : dev.query_sensors())
        {
            for (auto profile : sensor.get_stream_profiles())
            {
                if (profile.stream_type() == RS2_STREAM_GYRO)
                    found_gyro = true;

                if (profile.stream_type() == RS2_STREAM_ACCEL)
                    found_accel = true;
            }
        }
        if (found_gyro && found_accel)
            break;
    }
    return found_gyro && found_accel;
}

int main(int argc, char * argv[]) try
{
    // Before running the example, check that a device supporting IMU is connected
    if (!check_imu_is_supported())
    {
        std::cerr << "Device supporting IMU not found";
        return EXIT_FAILURE;
    }

    // Initialize window for rendering
    window app(1280, 720, "RealSense Motion Example");
    // Construct an object to manage view state
    glfw_state app_state(0.0, 0.0);
    // Register callbacks to allow manipulation of the view state
    register_glfw_callbacks(app, app_state);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    //rs2::pipeline pipe;
    //// Create a configuration for configuring the pipeline with a non default profile
    //rs2::config cfg;

    //// Add streams of gyro and accelerometer to configuration
    //cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    //cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    // Declare object for rendering camera motion
    camera_renderer camera;
    // Declare object that handles camera pose calculations
    rotation_estimator algo;

    //ImuSyncer sycner([&] (const imu_frame_t& imu_frame){
    //  rs2_vector gyro = {
    //    static_cast<float>(imu_frame.gyro.x()),
    //    static_cast<float>(imu_frame.gyro.y()),
    //    static_cast<float>(imu_frame.gyro.z())
    //  };
    //  rs2_vector accel = {
    //    static_cast<float>(imu_frame.accel.x()),
    //    static_cast<float>(imu_frame.accel.y()),
    //    static_cast<float>(imu_frame.accel.z())
    //  };
    //  algo.process_gyro(gyro, imu_frame.timestamp);
    //  algo.process_accel(accel);
    //});

    //// Start streaming with the given configuration;
    //// Note that since we only allow IMU streams, only single frames are produced
    //auto profile = pipe.start(cfg, [&](rs2::frame frame)
    //{
    //  sycner.AddMeasurement(frame);
    //});

    L515Async l515;
    // Main loop
    while (app)
    {
        const auto data = l515.GetSyncedData();
        for (const auto& imu_frame: data.imu_frames) {
          rs2_vector gyro = {
            static_cast<float>(imu_frame.gyro.x()),
            static_cast<float>(imu_frame.gyro.y()),
            static_cast<float>(imu_frame.gyro.z())
          };
          rs2_vector accel = {
            static_cast<float>(imu_frame.accel.x()),
            static_cast<float>(imu_frame.accel.y()),
            static_cast<float>(imu_frame.accel.z())
          };
          algo.process_gyro(gyro, imu_frame.timestamp);
          algo.process_accel(accel);
        }
        // Configure scene, draw floor, handle manipultation by the user etc.
        render_scene(app_state);
        // Draw the camera according to the computed theta
        camera.render_camera(algo.get_theta());
    }
    // Stop the pipeline
    //pipe.stop();

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
