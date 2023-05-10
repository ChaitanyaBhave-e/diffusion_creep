//* This file is part of the MOOSE framework
//* https://www.mooseframework.org
//*
//* All rights reserved, see COPYRIGHT for full restrictions
//* https://github.com/idaholab/moose/blob/master/COPYRIGHT
//*
//* Licensed under LGPL 2.1, please see LICENSE for details
//* https://www.gnu.org/licenses/lgpl-2.1.html

#include "ProjectionTensorMultiOP.h"
#include "RankTwoTensor.h"

registerMooseObject("TensorMechanicsApp", ProjectionTensorMultiOP);

InputParameters
ProjectionTensorMultiOP::validParams()
{
  InputParameters params = Material::validParams();
  params.addClassDescription(
      "Projection tensor of an interface defined by the gradients of the list of order parameters");
  params.addRequiredCoupledVar("all_etas",
                               "List of all order parameters for calculating projection tensor");
  params.addParam<MaterialPropertyName>("projection_tensor_name",
                                        "projection_tensor_ij",
                                        "Material property name for the i-j projection tensor");
  return params;
}

ProjectionTensorMultiOP::ProjectionTensorMultiOP(const InputParameters & parameters)
  : Material(parameters),
    _v(coupledValues("all_etas")),
    _grad_v(coupledGradients("all_etas")),
    _num_ops(coupledComponents("all_etas")),
    _projection_tensor(
        declareProperty<RankTwoTensor>(getParam<MaterialPropertyName>("projection_tensor_name")))
{
}

void
ProjectionTensorMultiOP::computeQpProperties()
{
  auto & S = _projection_tensor[_qp];
  S.zero();

  for (unsigned int i = 0; i < _num_ops; ++i)
  {
    // compute norm square of the order parameter gradients
    const Real grad_vii_norm_sq = ((*_grad_v[i])[_qp]).norm_sq();

    if (grad_vii_norm_sq > 0.0)
    {
      const Real kiix = ((*_grad_v[i])[_qp](0)) / std::sqrt(grad_vii_norm_sq);
      const Real kiiy = ((*_grad_v[i])[_qp](1)) / std::sqrt(grad_vii_norm_sq);
      const Real kiiz = ((*_grad_v[i])[_qp](2)) / std::sqrt(grad_vii_norm_sq);

      S(0, 0) += ((*_v[i])[_qp]) * (kiix * kiix);
      S(0, 1) += ((*_v[i])[_qp]) * (kiix * kiiy);
      S(1, 1) += ((*_v[i])[_qp]) * (kiiy * kiiy);
      S(0, 2) += ((*_v[i])[_qp]) * (kiix * kiiz);
      S(1, 2) += ((*_v[i])[_qp]) * (kiiy * kiiz);
      S(2, 2) += ((*_v[i])[_qp]) * (kiiz * kiiz);
    }
  }

  // fill in symmetrically
  S(1, 0) = S(0, 1);
  S(2, 0) = S(0, 2);
  S(2, 1) = S(1, 2);
}