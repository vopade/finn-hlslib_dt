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
  Bipolar(Bipolar const& o) : val(o.val) {} // copy constructor

  operator int() const {
    return  val? 1 : -1; // convert binary to bipolar for int
  }
  //Bipolar operator[](int tmp) const { 
  //  return val;
  //}
  // conversion from Bipolar to ap_int
  //ap_uint<1> operator ap_uint<1>() const { return  val? 1 : 0;}
  Bipolar operator*(Bipolar const& o) const {
    return ~(val ^ o.val); // XNOR, bitwise ~ can be used here since it is ap_uint<1> and it is more efficient than "!"
  }
/*
  ap_uint<1> operator+(Bipolar const& o) const {
    return val + o.val; // XNOR, bitwise ~ can be used here since it is ap_uint<1> and it is more efficient than "!"
  }
*/
Bipolar operator+=(Bipolar val, Bipolar const& o) const {
    return val = val + o.val; 
  }
};

#endif



