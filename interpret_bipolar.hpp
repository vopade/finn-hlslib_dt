#ifndef INTERPRET_BIPOLAR_HPP
#define INTERPRET_BIPOLAR_HPP

#include <ap_int.h>
#include <ostream>

class Bipolar {
  ap_uint<1>  val;
  template<unsigned> friend class BipolarAccu;
public:
  Bipolar (){};
  template<typename T> Bipolar(T val_) : val(val_) {}
  Bipolar(Bipolar const& o) : val(o.val) {}

  operator int() const {
    return  val? 1 : -1; // convert binary to bipolar for int
  }

  Bipolar operator*(Bipolar const& o) const {
    return ~(val ^ o.val); // XNOR, bitwise ~ can be used here since it is ap_uint<1> and it is more efficient than "!"
  }
};

#endif
