#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <iostream>

#include <popl.hpp>

#include <librealsense2/rs.hpp>

#include "cameras/l515.h"

int main(int argc, char* argv[]) {
  popl::OptionParser op;
  auto save_dir = op.add<popl::Value<std::string>>("o", "output", "directory to save the output");
  op.parse(argc, argv);

  L515Async l515;
  int cnt = 0;

  while (true) {
    const auto data = l515.GetSyncedData();
    cv::Mat bgr;
    cv::cvtColor(data.rgb_img, bgr, cv::COLOR_RGB2BGR);
    cv::imshow("rgb", bgr);
    int key = cv::waitKey(1);
    if (key == 'q') {
      break;
    } else if (key == 's') {
      std::string path = save_dir->value() + "/" + std::to_string(cnt++) + ".png";
      cv::imwrite(path, bgr);
      std::cout << "Saved to " << path << std::endl;
    }
  }
}
