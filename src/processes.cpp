#include <iostream>
#include <vector>
#include <unistd.h>  // sleep()
#include <cstdlib>   // system()

int main() {
    // Kiểm tra Terminator có tồn tại không
    if (std::system("which terminator > /dev/null 2>&1") != 0) {
        std::cerr << "Lỗi: Terminator chưa được cài đặt hoặc không tìm thấy!" << std::endl;
        return 1;
    }

    // Danh sách lệnh cần chạy
    std::vector<std::string> commands = {
        "cd ~/PX4-Autopilot && sudo chmod a+wr /dev/ttyUSB0",
        "cd ~/PX4-Autopilot && source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default && gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world",
        "cd ~/PX4-Autopilot && sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600"
    };

    // Lệnh mở Terminator
    std::string terminator_cmd = "terminator";

    // Mở tab đầu tiên
    terminator_cmd += " --new-tab -e \"bash -c '" + commands[0] + "; exec bash'\"";

    // Thêm các tab tiếp theo
    for (size_t i = 1; i < commands.size(); ++i) {
        terminator_cmd += " --new-tab -e \"bash -c '" + commands[i] + "; exec bash'\"";
    }

    // Chạy Terminator
    int ret = std::system(terminator_cmd.c_str());

    if (ret != 0) {
        std::cerr << "Lỗi khi chạy Terminator!" << std::endl;
    }

    return 0;
}
