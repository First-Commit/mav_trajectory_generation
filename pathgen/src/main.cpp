#include "pathgen_node.h"

int main(int argc, char **argv) {
  if(argc < 2) {
  	ROS_INFO("Must provide path file (eg. /path/to/path.csv)");
  	return -1;
  }
  std::string out_file;
  if(argc == 3) {
  	out_file = argv[2];
  }

  ros::init(argc, argv, "pathgen");

  // this will block
  PathGen pathgen(argv[1], out_file);

  return 0;
}
