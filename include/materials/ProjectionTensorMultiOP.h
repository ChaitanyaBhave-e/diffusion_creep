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

class ProjectionTensorMultiOP : public Material
{
public:
  static InputParameters validParams();
  ProjectionTensorMultiOP(const InputParameters & parameters);

protected:
  virtual void computeQpProperties() override;

  /// Global concentrations
  const std::vector<const VariableValue *> _v;
  const std::vector<const VariableGradient *> _grad_v;
  /// Order parameters vector
  const unsigned int _num_ops;

  // const std::vector<VariableName> _op_names;

  MaterialProperty<RankTwoTensor> & _projection_tensor;
};