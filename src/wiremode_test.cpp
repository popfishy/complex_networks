#include <math.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

const int width = 500;
const int heighth = 500;
const double rc = 100.0;
const double gama = -5;
const double beta = 0.8;
const double e = 0.1;
const double alfa = 0.8;

Mat picture_SF(width, heighth, CV_8UC3, Scalar(255, 255, 255, 0.5));
Mat picture_MB(width, heighth, CV_8UC3, Scalar(255, 255, 255, 0.5));
Mat picture_AB(width, heighth, CV_8UC3, Scalar(255, 255, 255, 0.5));
Mat picture_MA(width, heighth, CV_8UC3, Scalar(255, 255, 255, 0.5));
Mat picture_MY(width, heighth, CV_8UC3, Scalar(255, 255, 255, 0.5));

bool random(double p) {
  int N = 100000;
  int a = rand() % N + 1;
  if (p > static_cast<double>(a) / static_cast<double>(N))
    return true;
  else
    return false;
}

class uavNode {
 private:
  int ID;
  Point pos;
  bool iswired = false;

 public:
  vector<int> wirednodes;

  uavNode(int _ID) : ID(_ID) {
    int x = rand() % width;
    int y = rand() % heighth;
    pos = Point(x, y);
  }

  uavNode(int _ID, int x, int y) : ID(_ID) { pos = Point(x, y); }

  void all_connect(vector<uavNode>& nodes, int picture_flag) {
    if (nodes.empty()) {
      int radius = 5;
      Scalar circlecolor(50, 50, 123);
      switch (picture_flag) {
        case 0: {  // SF
          circle(picture_SF, pos, radius, circlecolor, -1, 8, 0);
          break;
        }
        case 1: {  // SF
          circle(picture_MB, pos, radius, circlecolor, -1, 8, 0);
          break;
        }
        case 2: {  // SF
          circle(picture_AB, pos, radius, circlecolor, -1, 8, 0);
          break;
        }
        case 3: {  // SF
          circle(picture_MA, pos, radius, circlecolor, -1, 8, 0);
          break;
        }
        case 4: {  // SF
          circle(picture_MY, pos, radius, circlecolor, -1, 8, 0);
          break;
        }
      }
    } else {
      for (int i = 0; i < nodes.size(); i++) {
        wire(nodes[i], picture_flag);
      }
    }
  }

  void connect(vector<uavNode>& nodes, int connectmode) {
    switch (connectmode) {
      case 0: {  // SF
        SF(nodes, connectmode);
        break;
      }
      case 1: {  // SF Mathbeney
        SF_MB(nodes, connectmode);
        break;
      }
      case 2: {  // SF Barrat
        SF_AB(nodes, connectmode);
        break;
      }
      case 3: {  // SF Manna
        SF_MA(nodes, connectmode);
        break;
      }
      case 4: {  // SF Manna
        SF_MY(nodes, connectmode);
        break;
      }
      default: {
        cout << "Please enter the right picture NO, range 0~4" << endl;
        cout << "This node's ID = " << ID << endl;
        cout << "This node's connectmode = " << connectmode << endl;
        break;
      }
    }
  }

  //网络中节点被连接的概率计算都采用的是几何概率，sum,chose,count 是关键，与 在
  //anylogic 中仿真类似
  void SF(vector<uavNode>& nodes, int picture_flag) {
    int sum_k = 0;
    for (int i = 0; i < nodes.size(); i++) sum_k += nodes[i].wirednodes.size();

    int chose = rand() % sum_k;
    int count = 0;

    for (int i = 0; i < nodes.size(); i++) {
      count += nodes[i].wirednodes.size();
      if (count > chose) {
        wire(nodes[i], picture_flag);
        break;
      }
    }
  }

  void SF_AB(vector<uavNode>& nodes, int picture_flag) {
    double sum = 0;

    for (int i = 0; i < nodes.size(); i++) {
      if (nodes[i].ID != ID &&
          find(nodes[i].wirednodes.begin(), nodes[i].wirednodes.end(), ID) ==
              nodes[i].wirednodes.end())
        sum += pow((double)nodes[i].wirednodes.size(), 0.5) *
               exp(get_distance(nodes[i]) / rc * (-1));
    }

    double chose = (double)(rand() % 100000) / (double)100000 * sum;
    double count = 0.0;

    for (int i = 0; i < nodes.size(); i++) {
      if (nodes[i].ID != ID &&
          find(nodes[i].wirednodes.begin(), nodes[i].wirednodes.end(), ID) ==
              nodes[i].wirednodes.end()) {
        count += pow((double)nodes[i].wirednodes.size(), 0.5) *
                 exp(get_distance(nodes[i]) / rc * (-1));
        if (count >= chose) {
          wire(nodes[i], picture_flag);
          break;
        }
      }
    }
  }

  void SF_MA(vector<uavNode>& nodes, int picture_flag) {
    double sum = 0;

    for (int i = 0; i < nodes.size(); i++) {
      if (nodes[i].ID != ID &&
          find(nodes[i].wirednodes.begin(), nodes[i].wirednodes.end(), ID) ==
              nodes[i].wirednodes.end())
        sum += pow((double)nodes[i].wirednodes.size(), beta) *
               pow(get_distance(nodes[i]), gama);
    }

    double chose = (double)(rand() % 100000) / (double)100000 * sum;
    double count = 0.0;

    for (int i = 0; i < nodes.size(); i++) {
      if (nodes[i].ID != ID &&
          find(nodes[i].wirednodes.begin(), nodes[i].wirednodes.end(), ID) ==
              nodes[i].wirednodes.end()) {
        count += pow((double)nodes[i].wirednodes.size(), beta) *
                 pow(get_distance(nodes[i]), gama);
        if (count >= chose) {
          wire(nodes[i], picture_flag);
          break;
        }
      }
    }
  }

  void SF_MB(vector<uavNode>& nodes, int picture_flag) {
    double sum = 0;

    for (int i = 0; i < nodes.size(); i++) {
      if (nodes[i].ID != ID &&
          find(nodes[i].wirednodes.begin(), nodes[i].wirednodes.end(), ID) ==
              nodes[i].wirednodes.end())
        sum += ((double)nodes[i].wirednodes.size()) *
               exp(get_distance(nodes[i]) / rc * (-1));
    }

    double chose = (double)(rand() % 100000) / (double)100000 * sum;
    double count = 0.0;

    for (int i = 0; i < nodes.size(); i++) {
      if (nodes[i].ID != ID &&
          find(nodes[i].wirednodes.begin(), nodes[i].wirednodes.end(), ID) ==
              nodes[i].wirednodes.end()) {
        count += ((double)nodes[i].wirednodes.size()) *
                 exp(get_distance(nodes[i]) / rc * (-1));
        if (count >= chose) {
          wire(nodes[i], picture_flag);
          break;
        }
      }
    }
  }

  void SF_MY(vector<uavNode>& nodes, int picture_flag) {
    double sum_k = 0.0;
    for (int i = 0; i < nodes.size(); i++) {
      if (nodes[i].ID != ID &&
          find(nodes[i].wirednodes.begin(), nodes[i].wirednodes.end(), ID) ==
              nodes[i].wirednodes.end() &&
          get_distance(nodes[i]) < rc) {
        double d = get_distance(nodes[i]);
        if (d < alfa * rc)
          sum_k += nodes[i].wirednodes.size() + e;
        else
          // TODO (rc * alfa * rc) --> (rc - alfa * rc) 应该是笔误
          sum_k +=
              (nodes[i].wirednodes.size() + e) * (rc - d) / (rc - alfa * rc);
      }
    }

    if (sum_k == 0.0) {
      circle(picture_MY, pos, 5, Scalar(50, 50, 123), -1, 8, 0);
      cout << "sum_k == 0, ID = " << ID << endl;
    } else {
      double chose = (double)(rand() % 100000) / (double)100000 * sum_k;
      double count = 0.0;

      for (int i = 0; i < nodes.size(); i++) {
        // find(nodes[i].wirednodes.begin(), nodes[i].wirednodes.end(), ID)
        // 若找不到ID则返回 nodes[i].wirednodes.end()
        if (nodes[i].ID != ID &&
            find(nodes[i].wirednodes.begin(), nodes[i].wirednodes.end(), ID) ==
                nodes[i].wirednodes.end() &&
            get_distance(nodes[i]) < rc) {
          double d = get_distance(nodes[i]);
          if (d < alfa * rc)
            count += nodes[i].wirednodes.size() + e;
          else
            count +=
                (nodes[i].wirednodes.size() + e) * (rc - d) / (rc - alfa * rc);
        }

        if (count > chose) {
          wire(nodes[i], picture_flag);
          break;
        }
      }
    }
  }

  void wire(uavNode& node, int picture_flag) {
    node.wirednodes.push_back(ID);
    iswired = true;
    wirednodes.push_back(node.get_ID());
    int radius = 5;
    int linewidth = 1;
    Scalar linecolor(123, 50, 50);
    Scalar redlinecolor(0, 0, 255);
    Scalar circlecolor(50, 50, 123);

    switch (picture_flag) {
      case 0: {  // SF
        line(picture_SF, pos, node.get_pos(), linecolor, linewidth, 8, 0);
        circle(picture_SF, pos, radius, circlecolor, -1, 8, 0);
        break;
      }
      case 1: {  // SF
        line(picture_MB, pos, node.get_pos(), linecolor, linewidth, 8, 0);
        circle(picture_MB, pos, radius, circlecolor, -1, 8, 0);
        break;
      }
      case 2: {  // SF
        line(picture_AB, pos, node.get_pos(), linecolor, linewidth, 8, 0);
        circle(picture_AB, pos, radius, circlecolor, -1, 8, 0);
        break;
      }
      case 3: {  // SF
        line(picture_MA, pos, node.get_pos(), linecolor, linewidth, 8, 0);
        circle(picture_MA, pos, radius, circlecolor, -1, 8, 0);
        break;
      }
      case 4: {  // SF
        line(picture_MY, pos, node.get_pos(), linecolor, linewidth, 8, 0);
        circle(picture_MY, pos, radius, circlecolor, -1, 8, 0);
        break;
      }
      default: {
        cout << "Please enter the right picture NO, range 0~4" << endl;
        break;
      }
    }
  }

  double get_distance(const uavNode& node) {
    return sqrt((pos.x - node.pos.x) * (pos.x - node.pos.x) +
                (pos.y - node.pos.y) * (pos.y - node.pos.y));
  }

  int get_ID() { return ID; }
  Point get_pos() { return pos; }
};

int main() {
  int N = 50;   // size of network's node
  int m0 = 2;  // initial size of nodes
  int xytxt = 1;  // if xytxt==0,不需要生成位置信息并保存到 TXT 文件，反之则需要

  vector<uavNode> nodes_SF;
  vector<uavNode> nodes_MB;
  vector<uavNode> nodes_AB;
  vector<uavNode> nodes_MA;
  vector<uavNode> nodes_MY;

  if (xytxt) {
    ofstream xy_txt("xy.txt");
    for (int i = 0; i < N; i++) {
      int x = rand() % width;
      int y = rand() % heighth;
      xy_txt << x << " " << y << endl;
    }
    xy_txt.close();
  }

  ifstream xy_txt("xy.txt");
  for (int i = 0; i < N; i++) {
    int x, y;
    xy_txt >> x >> y;
    if (i < m0) {
      uavNode node_SF(i, x, y);
      node_SF.all_connect(nodes_SF, 0);
      nodes_SF.push_back(node_SF);

      uavNode node_MY(i, x, y);
      node_MY.all_connect(nodes_MY, 4);
      nodes_MY.push_back(node_MY);
    } else {
      uavNode node_SF(i, x, y);
      node_SF.connect(nodes_SF, 0);
      nodes_SF.push_back(node_SF);

      uavNode node_MY(i, x, y);
      node_MY.connect(nodes_MY, 4);
      nodes_MY.push_back(node_MY);
    }

    uavNode node_MB(i, x, y);
    nodes_MB.push_back(node_MB);

    uavNode node_AB(i, x, y);
    nodes_AB.push_back(node_AB);

    uavNode node_MA(i, x, y);
    nodes_MA.push_back(node_MA);
  }
  xy_txt.close();

  for (int i = 1; i < N; i++) {
    if (i == 1) {
      nodes_MB[0].wire(nodes_MB[i], 1);
      nodes_AB[0].wire(nodes_AB[i], 2);
      nodes_MA[0].wire(nodes_MA[i], 3);
    } else {
      nodes_MB[i].connect(nodes_MB, 1);
      nodes_AB[i].connect(nodes_AB, 2);
      nodes_MA[i].connect(nodes_MA, 3);
      nodes_MY[i].connect(nodes_MY, 4);
    }
  }

  Rect rect(0, 0, 500, 500);
  rectangle(picture_SF, rect, Scalar(0, 0, 0), 3, 8, 0);
  rectangle(picture_MB, rect, Scalar(0, 0, 0), 3, 8, 0);
  rectangle(picture_AB, rect, Scalar(0, 0, 0), 3, 8, 0);
  rectangle(picture_MA, rect, Scalar(0, 0, 0), 3, 8, 0);
  rectangle(picture_MY, rect, Scalar(0, 0, 0), 3, 8, 0);

  imshow("SF", picture_SF);
  imshow("MB", picture_MB);
  imshow("MA", picture_MA);
  imshow("AB", picture_AB);
  imshow("MY", picture_MY);

  imwrite("picture_SF.jpg", picture_SF);
  imwrite("picture_MA.jpg", picture_MA);
  imwrite("picture_MB.jpg", picture_MB);
  imwrite("picture_AB.jpg", picture_AB);
  imwrite("picture_MY.jpg", picture_MY);

  waitKey(0);

  return 0;
}
