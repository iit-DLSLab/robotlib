/*******************************************************************************
*                                                       ,----,                 *
*                                                     .'   .' \                *
*                                                   ,----,'    |               *
*               ________  ___       ________        |    :  .  ;               *
*              |\   ___ \|\  \     |\   ____\       ;    |.'  /                *
*              \ \  \_|\ \ \  \    \ \  \___|_      `----'/  ;                 *
*               \ \  \ \\ \ \  \    \ \_____  \       /  ;  /                  *
*                \ \  \_\\ \ \  \____\|____|\  \     ;  /  /-,                 *
*                 \ \_______\ \_______\____\_\  \   /  /  /.`|                 *
*                  \|_______|\|_______|\_________\./__;      :                 *
*                                     \|_________||   :    .'                  *
*                                                 ;   | .'                     *
*                                                 `---'                        *
********************************************************************************
* Author:            Legacy Code                                               *
* Maintainer:        Hendrik de Bruin                                          *
* Maintainer email:  hendrik.debruin@iit.it                                    *
*******************************************************************************/
#ifndef _ROBOTLIB_HYQ_JSIM_HPP_
#define _ROBOTLIB_HYQ_JSIM_HPP_

#include "robotlib/rbd/rbd.h"
#include "robotlib/rbd/StateDependentMatrix.h"

#include "robotlib/base/declarations.hpp"
#include "robotlib/base/jsim.hpp"
#include "robotlib/hyq/transforms.hpp"
#include "robotlib/hyq/inertia_properties.hpp"


namespace dls {
namespace dog {
namespace HyQ {
namespace dyn {

/**
 * The type of the Joint Space Inertia Matrix (JSIM) of the robot HyQ.
 */
class JSIM : public dog::JSIMBase
{
private:
	typedef dls::rbd::StateDependentMatrix<dog::JointState, 18, 18, JSIM> Base;
public:
	typedef Base::Scalar Scalar;
	typedef Base::Index Index;
	typedef Eigen::Matrix<double,18,18> MatrixType;
	/** The type of the F sub-block of the floating-base JSIM */
	typedef const Eigen::Block<const MatrixType,6,12> BlockF_t;
	/** The type of the fixed-base sub-block of the JSIM */
	typedef const Eigen::Block<const MatrixType,12,12> BlockFixedBase_t;
public:
	JSIM(InertiaProperties&, ForceTransforms&);
	~JSIM() {}

	const JSIM& update(const dog::JointState&);


	/**
	 * Computes and saves the matrix L of the L^T L factorization of this JSIM.
	 */
	void computeL();
	/**
	 * Computes and saves the inverse of this JSIM.
	 * This function assumes that computeL() has been called already, since it
	 * uses L to compute the inverse. The algorithm takes advantage of the branch
	 * induced sparsity of the robot, if any.
	 */
	void computeInverse();
	/**
	 * Returns an unmodifiable reference to the matrix L. See also computeL()
	 */
	const MatrixType& getL() const;
	/**
	 * Returns an unmodifiable reference to the inverse of this JSIM
	 */
	const MatrixType& getInverse() const;

	/**
	 * The spatial composite-inertia tensor of the robot base,
	 * ie the inertia of the whole robot for the current configuration.
	 * According to the convention of this class about the layout of the
	 * floating-base JSIM, this tensor is the 6x6 upper left corner of
	 * the JSIM itself.
	 * \return the 6x6 InertiaMatrix that correspond to the spatial inertia
	 *   tensor of the whole robot, according to the last joints configuration
	 *   used to update this JSIM
	 */
	const InertiaMatrix& getWholeBodyInertia() const;
	/**
	 * The matrix that maps accelerations in the actual joints of the robot
	 * to the spatial force acting on the floating-base of the robot.
	 * This matrix is the F sub-block of the JSIM in Featherstone's notation.
	 * \return the 6x12 upper right block of this JSIM
	 */
	const BlockF_t getF() const;
	/**
	 * The submatrix of this JSIM related only to the actual joints of the
	 * robot (as for a fixed-base robot).
	 * This matrix is the H sub-block of the JSIM in Featherstone's notation.
	 * \return the 12x12 lower right block of this JSIM,
	 *   which correspond to the fixed-base JSIM
	 */
	const BlockFixedBase_t getFixedBaseBlock() const;
protected:
	/**
	 * Computes and saves the inverse of the matrix L. See also computeL()
	 */
	void computeLInverse();
private:
	InertiaProperties& linkInertias;
	ForceTransforms* frcTransf;

	// The composite-inertia tensor for each link
	InertiaMatrix Ic_trunk;
	InertiaMatrix Ic_LF_hipassembly;
	InertiaMatrix Ic_LF_upperleg;
	const InertiaMatrix& Ic_LF_lowerleg;
	InertiaMatrix Ic_RF_hipassembly;
	InertiaMatrix Ic_RF_upperleg;
	const InertiaMatrix& Ic_RF_lowerleg;
	InertiaMatrix Ic_LH_hipassembly;
	InertiaMatrix Ic_LH_upperleg;
	const InertiaMatrix& Ic_LH_lowerleg;
	InertiaMatrix Ic_RH_hipassembly;
	InertiaMatrix Ic_RH_upperleg;
	const InertiaMatrix& Ic_RH_lowerleg;

	MatrixType L;
	MatrixType Linv;
	MatrixType inverse;
};


} // dyn
} // HyQ
} // dog
} // dls
#endif // _ROBOTLIB_HYQ_JSIM_HPP_
