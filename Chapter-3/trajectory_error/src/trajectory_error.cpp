#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;
typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
// path to trajectory file
string estimated_file = "../src/estimated.txt";
string groundtruth_file = "../src/groundtruth.txt";
// function for plotting trajectory, don't edit this code
// start point is red and end point is blue

TrajectoryType ReadTrajectory(string file_path);
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

int main(int argc, char **argv) {
    // implement pose reading code
    // start your code here (5~10 lines)
    TrajectoryType gt_poses = ReadTrajectory(groundtruth_file);
    TrajectoryType esti_poses = ReadTrajectory(estimated_file);
    if(gt_poses.size() == 0 || esti_poses.size() == 0){
        cerr << "Trajectory is empty!" << endl;
    }
    if(gt_poses.size() != esti_poses.size()){
        cerr << "Two Trajectories have different sizes!" << endl;
    }
    // compute error
    double rmse = 0;
    for (size_t i = 0; i < esti_poses.size(); i++) {
        Sophus::SE3d p1 = esti_poses[i], p2 = gt_poses[i];
        double error = (p2.inverse() * p1).log().norm();
        rmse += error * error;
    }
    rmse = rmse / double(esti_poses.size());
    rmse = sqrt(rmse);
    cout << "RMSE Error is " << rmse << endl;

    // draw error
    DrawTrajectory(gt_poses, esti_poses);

    // end your code here

    return 0;
}
TrajectoryType ReadTrajectory(string file_path){
    TrajectoryType poses;
    ifstream file_input(file_path);
    if(!file_input){
        cerr << "Trajectory " << file_path << " not found." << endl;
    }

    while(!file_input.eof()){
        double time, tx, ty, tz, qx, qy, qz, qw;
        file_input >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3d p(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        poses.push_back(p);
    }
    file_input.close();
    return poses;
}
/*******************************************************************************************/
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {
    if (gt.empty()) {
        cerr << "Groundtruth trajectory is empty!" << endl;
        return;
    }

    if (esti.empty()) {
        cerr << "Estimated trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        // groundtruth
        for (size_t i = 0; i < gt.size() - 1; i++) {
            glColor3f(0.0f, 1.0f, 0.0f);  // green
            glBegin(GL_LINES);
            auto p1 = gt[i], p2 = gt[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        // estimated
        for (size_t i = 0; i < esti.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f); // red
            glBegin(GL_LINES);
            auto p1 = esti[i], p2 = esti[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}