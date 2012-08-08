#include <iostream>
#include <vector>

using namespace std;

struct point {
  double x;
  double y;
};

vector<point> pt;
//void test(vector<point>& pt) {
void test(double *pt) {
  pt[1] = 0.896;
}

int main() {
  pt.resize(2);
//  pt[1].x = 5;
//  pt[1].y = 34;
//  cout << &pt[1].x << ' ' << &pt[1].y << endl;
//  cout << pt[1].x << ' ' << pt[1].y << endl;
//  test(pt);
  double pt[3] = {0.564, 0.343, 0.111};
  test(pt);
  cout << pt[0] << ' ' << pt[1] << endl;
  return 0;
}
