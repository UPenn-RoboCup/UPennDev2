#include <iostream>
#include <vector>

using namespace std;

struct point {
  double x;
  double y;
};

vector<point> pt;
//void test(vector<point>& pt) {
void test(void) {
  point pt2;
  pt2.x = 5.0;
  pt2.y = 10.0;
  pt[1] = pt2;
}

int main() {
  pt.resize(2);
//  pt[1].x = 5;
//  pt[1].y = 34;
//  cout << &pt[1].x << ' ' << &pt[1].y << endl;
//  cout << pt[1].x << ' ' << pt[1].y << endl;
//  test(pt);
  test();
  cout << &pt[1].x << ' ' << &pt[1].y << endl;
  cout << pt[1].x << ' ' << pt[1].y << endl;
  cout << &pt[2].x << ' ' << &pt[2].y << endl;
  cout << pt[2].x << ' ' << pt[2].y << endl;
  return 0;
}
