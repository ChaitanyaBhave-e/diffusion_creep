//* This file is part of the MOOSE framework
//* https://www.mooseframework.org
//*
//* All rights reserved, see COPYRIGHT for full restrictions
//* https://github.com/idaholab/moose/blob/master/COPYRIGHT
//*
//* Licensed under LGPL 2.1, please see LICENSE for details
//* https://www.gnu.org/licenses/lgpl-2.1.html

#include "IsotropicDiffusionTensor.h"

registerMooseObject("PhaseFieldApp", IsotropicDiffusionTensor);

InputParameters
IsotropicDiffusionTensor::validParams()
{
    InputParameters params = Material::validParams();
    params.addClassDescription("Generates an isotropic diffusion tensor");
    params.addParam<MaterialPropertyName>("diffusion_coefficient",
                                        "D",
                                        "Material property name for the isotropic diffusion coefficient value");

    params.addParam<std::string>(
      "diffusivity_tensor_name", "D_tensor", "Name for the diffusivity tensor material property");
    return params;
}

IsotropicDiffusionTensor::IsotropicDiffusionTensor(const InputParameters & parameters):
    DerivativeMaterialInterface<Material>(parameters),
    _diff_coeff(getMaterialProperty<Real>("diffusion_coefficient")),
    _D(declareProperty<RealTensorValue>(getParam<std::string>("diffusivity_tensor_name")) )
{
}

void
IsotropicDiffusionTensor::computeQpProperties()
{
    RealTensorValue I(1, 0, 0, 0, 1, 0, 0, 0, 1);
    _D[_qp] = I * _diff_coeff[_qp];
}