#include "utils/CVUtils.hpp"
#include "utils/MathUtils.hpp"

using namespace std;
using namespace VForce;
using namespace Utils;

int main(int argc, char **argv) {
  Eigen::Matrix4f tf;
  float x, y, z, R, P, Y;
  cin >> x >> y >> z >> R >> P >> Y;
  pose2matrix(x / 1000.f, y / 1000.f, z / 1000.f, deg2rad(R), deg2rad(P), deg2rad(Y), tf);
  cout << tf.format(IOF) << endl;
  return 0;
}
