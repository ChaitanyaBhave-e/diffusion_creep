//* This file is part of the MOOSE framework
//* https://www.mooseframework.org
//*
//* All rights reserved, see COPYRIGHT for full restrictions
//* https://github.com/idaholab/moose/blob/master/COPYRIGHT
//*
//* Licensed under LGPL 2.1, please see LICENSE for details
//* https://www.gnu.org/licenses/lgpl-2.1.html

#include "ProjectionTensor2OP.h"
#include "RankTwoTensor.h"

registerMooseObject("TensorMechanicsApp", ProjectionTensor2OP);

InputParameters
ProjectionTensor2OP::validParams()
{
  InputParameters params = Material::validParams();
  params.addClassDescription(
      "Projection tensor of an interface defined by the gradients of the two order parameter");
  params.addCoupledVar("vi", "An order parameter of the interface, assumed to vary from 0 to 1.");
  params.addCoupledVar("vj", "Second order parameter of the interface, assumed to vary from 0 to 1.");
  params.addParam<MaterialPropertyName>("projection_tensor_name",
                                        "projection_tensor_ij",
                                        "Material property name for the i-j projection tensor");
  return params;
}

ProjectionTensor2OP::ProjectionTensor2OP(const InputParameters & parameters)
  : Material(parameters),
    _vi(coupledValue("vi")),
    _vj(coupledValue("vj")),
    _grad_vi(coupledGradient("vi")),
    _grad_vj(coupledGradient("vj")),
    _projection_tensor(
        declareProperty<RankTwoTensor>(getParam<MaterialPropertyName>("projection_tensor_name")))
{
}

void
ProjectionTensor2OP::computeQpProperties()
{
  auto & S = _projection_tensor[_qp];
  S.zero();

  // compute norm square of the order parameter gradients
  const Real grad_vij_norm_sq = (_grad_vi[_qp] - _grad_vj[_qp]).norm_sq();

  const Real kijx = (_grad_vi[_qp](0) - _grad_vj[_qp](0))/std::sqrt(grad_vij_norm_sq);
  const Real kijy = (_grad_vi[_qp](1) - _grad_vj[_qp](1))/std::sqrt(grad_vij_norm_sq);
  const Real kijz = (_grad_vi[_qp](2) - _grad_vj[_qp](2))/std::sqrt(grad_vij_norm_sq);

  S(0, 0) += (1.0 - kijx * kijx);
  S(0, 1) += -kijx * kijy;
  S(1, 1) += (1.0 - kijy * kijy);
  S(0, 2) += -kijx * kijz;
  S(1, 2) += -kijy * kijz;
  S(2, 2) += (1.0 - kijz * kijz);

  // fill in symmetrically
  S(1, 0) = S(0, 1);
  S(2, 0) = S(0, 2);
  S(2, 1) = S(1, 2);
}
