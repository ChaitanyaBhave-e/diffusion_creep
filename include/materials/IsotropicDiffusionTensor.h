//* This file is part of the MOOSE framework
//* https://www.mooseframework.org
//*
//* All rights reserved, see COPYRIGHT for full restrictions
//* https://github.com/idaholab/moose/blob/master/COPYRIGHT
//*
//* Licensed under LGPL 2.1, please see LICENSE for details
//* https://www.gnu.org/licenses/lgpl-2.1.html

#pragma once

#include "Material.h"
#include "DerivativeMaterialInterface.h"

class IsotropicDiffusionTensor : public DerivativeMaterialInterface<Material>
{
public:
  static InputParameters validParams();

  IsotropicDiffusionTensor(const InputParameters & parameters);
  virtual void computeQpProperties();

protected:
  ///Material property declarations
  const MaterialProperty<Real> & _diff_coeff;
  MaterialProperty<RealTensorValue> & _D;
};
