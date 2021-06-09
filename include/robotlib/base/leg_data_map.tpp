#include <iostream>
namespace dls
{
namespace robot
{

template<typename T> inline
LegDataMap<T>::LegDataMap(const T& defaultValue) {
    assignAll(defaultValue);
}

template<typename T> inline
LegDataMap<T>::LegDataMap(const LegDataMap& rhs) //cannot use initializer list for arrays?
{
    copydata(rhs);
}

template<typename T> inline
LegDataMap<T>& LegDataMap<T>::operator=(const LegDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LegDataMap<T>& LegDataMap<T>::operator=(const T& defaultValue)
{
    assignAll(defaultValue);
    return *this;
}

template<typename T> inline
T& LegDataMap<T>::operator[](LegID hip) {
    return data[hip];
}

template<typename T> inline
T& LegDataMap<T>::operator[](int index) throw(std::runtime_error) {
    if(index<0 || index>_LEGS_COUNT) {
        throw(std::runtime_error("Leg index out of bounds"));
    }
    return data[index];
}

template<typename T> inline
const T& LegDataMap<T>::operator[](LegID hip) const {
    return data[hip];
}

template<typename T> inline
const T& LegDataMap<T>::operator[](int index) const throw(std::runtime_error) {
    if(index<0 || index>_LEGS_COUNT) {
        throw(std::runtime_error("Leg index out of bounds"));
    }
    return data[index];
}

template<typename T> inline
void LegDataMap<T>::copydata(const LegDataMap& rhs) {
    data[LF] = rhs[LF];
    data[RF] = rhs[RF];
    data[LH] = rhs[LH];
    data[RH] = rhs[RH];
}

template<typename T> inline
void  LegDataMap<T>::assignAll(const T& value) {
    data[LF] = value;
    data[RF] = value;
    data[LH] = value;
    data[RH] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LegDataMap<T>& map) {
    out << "LF = " << map[LF] << "  RF = " << map[RF];
    out << "  LH = " << map[LH] << "  RH = " << map[RH];
    return out;
}

//operator overload defined out of the class legdatamap
template<typename T> inline
LegDataMap<T> operator+(const LegDataMap<T>& lhs, const LegDataMap<T>& rhs) {
    LegDataMap<T> out;
    out[LF] = lhs[LF] + rhs[LF];
    out[RF] = lhs[RF] + rhs[RF];
    out[LH] = lhs[LH] + rhs[LH];
    out[RH] = lhs[RH] + rhs[RH];
    return out;
}

template<typename T> inline
LegDataMap<T> operator-(const LegDataMap<T>& lhs, const LegDataMap<T>& rhs) {
    LegDataMap<T> out;
    out[LF] = lhs[LF] - rhs[LF];
    out[RF] = lhs[RF] - rhs[RF];
    out[LH] = lhs[LH] - rhs[LH];
    out[RH] = lhs[RH] - rhs[RH];
    return out;
}
} // namespace robot
} // namespace dls

