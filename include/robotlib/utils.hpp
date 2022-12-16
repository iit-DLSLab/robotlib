#ifndef _ROBOTLIB_UTILS_HPP_
#define _ROBOTLIB_UTILS_HPP_

#include <memory>
#include <urdf_model/pose.h>
#include <Eigen/Dense>

namespace robotlib
{
    /**
        * Useful functions or data structures
        */
    template <class Data>
    struct Iterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = Data;
        using pointer = Data *;
        using reference = Data &;
        Iterator(pointer ptr) : m_ptr(ptr) {}
        reference operator*() const { return *m_ptr; }
        pointer operator->() { return m_ptr; }
        Iterator &operator++()
        {
            m_ptr++;
            return *this;
        }
        Iterator operator++(int)
        {
            Iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        Iterator &operator--()
        {
            m_ptr--;
            return *this;
        }

        Iterator operator--(int)
        {
            Iterator tmp = *this;
            --(*this);
            return tmp;
        }

        friend bool operator==(const Iterator &a, const Iterator &b) { return a.m_ptr == b.m_ptr; };
        friend bool operator!=(const Iterator &a, const Iterator &b) { return a.m_ptr != b.m_ptr; };

    protected:
        pointer m_ptr;
    };

    template <class Data>
    class ContainerBase
    {
    public:
        ContainerBase();
        virtual ~ContainerBase();

        virtual Iterator<const Data> begin() const = 0;
        virtual Iterator<const Data> end() const = 0;

        //TODO: define [] operator using a string as input (it implies that all the possible Data classes have a getName function)

        virtual int size() const = 0;
    };

    template <class Data, unsigned int N>
    class Container : public ContainerBase<Data>
    {
    public:
        Container(const std::array<Data, N> data);

        virtual ~Container();

        virtual Iterator<const Data> begin() const override;
        virtual Iterator<const Data> end() const override;

        virtual int size() const override;

    protected:
        const std::array<Data, N> data_;
    };

    /**
     * @brief converts a quaternion \f$\mathbf{q}\f$ into a rotation matrix
     * \f$R_q(\mathbf{q})\f$. The rotation matrix maps a vector
     * \f$\mathbf{z}\in\mathbb{R}^{3\times1}\f$ (expressed in global coordinates)
     * into a vector \f$ \mathbf{z}' \in \mathbb{R}^{3\times1}\f$ (expressed in
     * local coordinates), such that \f$ \mathbf{z}' = R_q(\mathbf{q})\mathbf{z}\f$.
     *
     * @param[in] q structure containing the quaternion
     * @return the 3 by 3 rotation matrix \f$R_q(\mathbf{q})\f$
     * @remark the function uses the formula (125) from <a href="https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf">"Representing Attitude: Euler
     *  Angles, Unit Quaternions, and Rotation Vectors"</a> by James Diebel.
     * @date July 2005
     */
    inline Eigen::Matrix3d quatToRotMat(const Eigen::Quaterniond & q);

    /*!
    *@brief Get Eigen matrix from urdf pose.
    *@param[in] urdf_pose pose of type urdf::Pose.
    *@return 4x4 eigen matrix.
    */
    Eigen::Matrix4d get_eigen_matrix4d_from_urdf_pose(urdf::Pose urdf_pose);

} // namespace robotlib

#include "utils.tpp"

#endif // _ROBOTLIB_UTILS_HPP_