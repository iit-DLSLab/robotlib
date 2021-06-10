/*
#include <iostream>
namespace dls
{
namespace robot
{


template<class Robot<unsigned int NLEGS>,class Data> inline
LegDataMap<Robot<NLEGS>,Data>::LegDataMap(const Data& defaultValue) {
    assignAll(defaultValue);
}
/*
template<typename Data> inline
LegDataMap<Data>::LegDataMap(const LegDataMap& rhs) //cannot use initializer list for arrays?
{
    copydata(rhs);
}

template<typename Data> inline
LegDataMap<Data>& LegDataMap<Data>::operator=(const LegDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename Data> inline
LegDataMap<Data>& LegDataMap<Data>::operator=(const Data& defaultValue)
{
    assignAll(defaultValue);
    return *this;
}
*/
template<class Robot<unsigned int NLEGS>, class Data> inline
Data& LegDataMap<Robot<NLEGS>, Data>::operator[](int index) throw(std::runtime_error) {
    if(index<0 || index>data_.size()) {
        throw(std::runtime_error("Leg index out of bounds"));
    }
    return data_[index];
}

/*

template<typename Data> inline
const Data& LegDataMap<Data>::operator[](int index) const throw(std::runtime_error) {
    if(index<0 || index>data.size()) {
        throw(std::runtime_error("Leg index out of bounds"));
    }
    return data[index];
}

template<typename Data> inline
void LegDataMap<Data>::copydata(const LegDataMap& rhs) {
    std::array<std::Data, NLEGS>::iterator itlegs;
    for (itlegs=legs.begin();itlegs<legs.end();++itlegs){
        std::cout << (*itlegs)->getName() <<'\n';
    }
}
*/
template<class Robot<unsigned int NLEGS>, class Data> inline
void  LegDataMap<Robot<NLEGS>, Data>::assignAll(const Data& value) {
    typename std::array<Data, NLEGS>::iterator itData;
    for (itData=data_.begin();itData<data_.end();++itData){
         *itData = value;
    }
}
/*
template<typename Data> inline
std::ostream& operator<<(std::ostream& out, const LegDataMap<Data>& map) {
    out << "LF = " << map[LF] << "  RF = " << map[RF];
    out << "  LH = " << map[LH] << "  RH = " << map[RH];
    return out;
}

//operator overload defined out of the class legdatamap
template<typename Data> inline
LegDataMap<Data> operator+(const LegDataMap<Data>& lhs, const LegDataMap<Data>& rhs) {
    LegDataMap<Data> out;
    out[LF] = lhs[LF] + rhs[LF];
    out[RF] = lhs[RF] + rhs[RF];
    out[LH] = lhs[LH] + rhs[LH];
    out[RH] = lhs[RH] + rhs[RH];
    return out;
}

template<typename Data> inline
LegDataMap<Data> operator-(const LegDataMap<Data>& lhs, const LegDataMap<Data>& rhs) {
    LegDataMap<Data> out;
    out[LF] = lhs[LF] - rhs[LF];
    out[RF] = lhs[RF] - rhs[RF];
    out[LH] = lhs[LH] - rhs[LH];
    out[RH] = lhs[RH] - rhs[RH];
    return out;
}
*/
} // namespace robot
} // namespace dls

*/