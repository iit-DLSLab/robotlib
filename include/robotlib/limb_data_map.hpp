#ifndef _ROBOTLIB_LIMB_DATA_MAP_HPP_
#define _ROBOTLIB_LIMB_DATA_MAP_HPP_

#include "limb_base.hpp"
#include "data_map.hpp"

namespace robotlib
{
    /*!
     * @brief LimbDataMap class.
     * @details
     * This templated class is used to store data for each limb.
     * @tparam Data class of the data associated to limbs.
    */
    template <class Data>
    class LimbDataMap : public DataMap<LimbBase, Data>
    {
        //! RobotBase is a friend class to let it use the private costructor of the LimbDataMap class.
        friend class RobotBase;

    public:
        LimbDataMap(const LimbDataMap& limb_data_map);

        /*!
         * @brief Destructor.
         */
        virtual ~LimbDataMap();
        
        using DataMap<LimbBase, Data>::operator=;
        using DataMap<LimbBase, Data>::operator[];
        
        LimbDataMap<Data>& operator=(const LimbDataMap<Data>&);
        // LimbDataMap<Data>& operator=(const ContainerBase<std::shared_ptr<Data>>&);

        // operator std::vector<Data>() const;

        // LimbDataMap<Data>& operator<< (Data);
        // LimbDataMap<Data>& operator, (Data);

        // /// TODO: Print in new line if data is a vector, matrix, etc... in same line of limb name (as for JointState) if data is a single value
        // /*!
        //  * @brief Print LimbDataMap information.
        //  */
        // void print() const;

    protected:

        LimbDataMap(const ContainerBase<LimbBase>&, const Data&);
        LimbDataMap(const ContainerBase<LimbBase>&, const std::vector<Data>&);

    // private:
    //     Iterator<typename DataMap<LimbBase, Data>::DataPair> dataIt;
    };
}

#include "limb_data_map.tpp"

#endif //_ROBOTLIB_LIMB_DATA_MAP_HPP_