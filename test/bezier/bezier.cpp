extern "C" {
  // auto bezier(float p0_x, float p0_y, float p1_x, float p1_y, float p2_x, float p2_y) {
  //   return [=](float t) {
  //     float t_prime = 1 - t;
  //     float t1 = t_prime * t_prime, t2 = 2 * t_prime * t, t3 = t * t;
  //     return std::pair<float, float>(
  //       t1 * p0_x + t2 * p1_x + t3 * p2_x,
  //       t1 * p0_y + t2 * p1_y + t3 * p2_y
  //     );
  //   };
  // }

  // void* CreateSet() {
  //   return new std::set<int>({ 2, 12, 13, 14, 15 });
  // }

  // int GetFromSet(void* set_ptr) {
  //   std::set<int> * set = reinterpret_cast<std::set<int> *>(set_ptr);
  //   return *(set->begin());
  // }
};

extern "C" void* CreateN() {
  return new int(1);
}

extern "C" int GetN(void* ptr) {
  return *((int*)ptr);
}
