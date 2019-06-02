//
// Created by eirik on 12.05.19.
//

vector<double> compute3PtPlaneEq(std::vector<cv::Vec3d> &tvecs)
{
    double a1 = tvecs[1][0] - tvecs[0][0];
    double b1 = tvecs[1][1] - tvecs[0][1];
    double c1 = tvecs[1][2] - tvecs[0][2];

    double a2 = tvecs[2][0] - tvecs[0][0];
    double b2 = tvecs[2][1] - tvecs[0][1];
    double c2 = tvecs[2][2] - tvecs[0][2];

    double a = b1 * c2 - b2 * c1;
    double b = a2 * c1 - a1 * c2;
    double c = a1 * b2 - b1 * a2;
    double d = (- a * tvecs[0][0] - b * tvecs[0][1] - c * tvecs[0][2]);

    std::cout << std::fixed;
    std::cout << "Plane equation: " << a << " x + " << b
              << " y + " << c << " z + " << d << " = 0.";

    return (a,b,c,d);
}

void pointToPlane(std::vector<double> planeEq, std::vector<cv::Vec3d> points3d){

    constexpr double n = [planeEq[0], planeEq[1], planeEq[2]];
    constexpr double  p = n * planeEq[3];

    for (int i =0; i<3;i++){
        dist += (points3d[0][i]-p[i])*n[i];
    }

    std::cout << n << std::endl;
}