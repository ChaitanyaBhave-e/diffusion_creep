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
#include "RankTwoTensorForward.h"

class ProjectionTensor2OP : public Material
{
public:
  static InputParameters validParams();

  ProjectionTensor2OP(const InputParameters & parameters);

protected:
  virtual void computeQpProperties() override;

  /// Order parameters i, j
  const VariableValue & _vi;
  const VariableValue & _vj;
  /// Gradient of order parameters
  const VariableGradient & _grad_vi;
  const VariableGradient & _grad_vj;

  MaterialProperty<RankTwoTensor> & _projection_tensor;
};
