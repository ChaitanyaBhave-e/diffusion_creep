[Mesh]
  type = GeneratedMesh
  dim = 2
  nx = '${fparse int ( 2 * xmax / ${GlobalParams/int_width} ) }'
  ny = '${fparse int ( 2 * ymax / ${GlobalParams/int_width} ) }'
  xmin = 0
  xmax = 1000.0
  ymin = 0
  ymax = 1000.0
  uniform_refine = 2
  skip_partitioning = true
[]

[GlobalParams]
  int_width = 20.0
  file_base_name = 'two_grain/two_grain_'
  displacements = 'disp_x disp_y'
  derivative_order = 2
[]
[Variables]
  [mu]
  []
  [c_v]
    initial_condition = 2.2877916582962764e-7
  []
  [jx]
  []
  [jy]
  []
  [disp_x]
  []
  [disp_y]
  []
[]
[AuxVariables]
  [s11_aux]
    order = CONSTANT
    family = MONOMIAL
  []
  [s22_aux]
    order = CONSTANT
    family = MONOMIAL
  []
  [s33_aux]
    order = CONSTANT
    family = MONOMIAL
  []
  [eta0]
  []
  [eta1]
  []
  [eta2]
  []
  [bnds]
  []
  [gb]
    order = CONSTANT
    family = MONOMIAL
  []
[]
[ICs]
  [eta0_grain]
    type = BoundingBoxIC
    variable = eta0
    x1 = ${Mesh/xmin}
    x2 = '${fparse ${Mesh/xmax} /2 }'
    y1 = '${fparse ${Mesh/ymin} }'
    y2 = '${fparse ${Mesh/ymax}  }'
    inside = 1
    outside = 0
  []
  [eta1_grain]
    type = BoundingBoxIC
    variable = eta1
    x1 = '${fparse ${Mesh/xmax} /2 }'
    x2 = ${Mesh/xmax}
    y1 = '${fparse ${Mesh/ymin}}'
    y2 = '${fparse ${Mesh/ymax}}'
    inside = 1
    outside = 0
  []
  [eta2_grain]
    type = BoundingBoxIC
    variable = eta2
    x1 = ${Mesh/xmin}
    x2 = ${Mesh/xmax}
    y1 = ${Mesh/ymin}
    y2 = ${Mesh/ymax}
    inside = 0
    outside = 1
  []
[]
[BCs]
  [bottom_y]
    type = DirichletBC
    variable = disp_y
    boundary = bottom
    value = 0
  []
  [Pressure]
    [top]
      boundary = top
      function = pressure_func
    []
  []
  # [gb_c_v]
  #   type = DirichletBC
  #   variable = c_v
  #   value = 2.2877916582962764e-7
  #   boundary = 'top left bottom right'
  # []
[]
[Functions]
  [pressure_func]
    type = ParsedFunction
    symbol_names = 'P0'
    symbol_values = '${fparse 50 * 10^6 * 6.242e-9}'
    expression = -P0
  []
[]
# [Modules/TensorMechanics/Master]
#   [all]
#     add_variables = true
#     incremental = true
#     strain = SMALL
#   []
# []
[Kernels]
  [TensorMechanics]
    displacements = 'disp_x disp_y'
  []
  #Vacancy diffusion
  [dc_dt]
    type = TimeDerivative
    variable = c_v
  []
  [conc]
    type = CHSplitConcentration
    variable = c_v
    mobility = M_v
    chemical_potential_var = mu
  []
  [GB_sink]
    type = MaskedBodyForce
    variable = c_v
    # value = -1
    mask = 'gb_relax_prefactor'
    coupled_variables = 'gb mu'
  []
  [chempot]
    type = CHSplitChemicalPotential
    variable = mu
    chemical_potential_prop = mu_v_prop
    c = c_v
  []
  [flux_x]
    type = CHSplitFlux
    variable = jx
    component = '0'
    mobility_name = M_v
    mu = mu
    c = c_v
  []
  [flux_y]
    type = CHSplitFlux
    variable = jy
    component = '1'
    mobility_name = M_v
    mu = mu
    c = c_v
  []
[]
[AuxKernels]
  [matl_s11]
    type = RankTwoAux
    rank_two_tensor = stress
    index_i = 0
    index_j = 0
    variable = s11_aux
  []
  [matl_s22]
    type = RankTwoAux
    rank_two_tensor = stress
    index_i = 1
    index_j = 1
    variable = s22_aux
  []
  [matl_s33]
    type = RankTwoAux
    rank_two_tensor = stress
    index_i = 2
    index_j = 2
    variable = s33_aux
  []
  [bnds_aux]
    type = BndsCalcAux
    variable = bnds
    v = 'eta0 eta1 eta2'
  []
  [gb_val]
    type = MaterialRealAux
    variable = gb
    property = gb_mat
  []
[]
[Materials]
  [constants]
    type = GenericConstantMaterial
    prop_names = 'kB T V_a R_gb'
    prop_values = '8.6173324e-5 973 0.01099207207 1.1' #R_gb = 1.1 if no BC on c_v
  []
  [gb_mat]
    type = DerivativeParsedMaterial
    coupled_variables = 'eta0 eta1 eta2 bnds'
    property_name = 'gb_mat'
    # expression = '16*(eta0^2*eta1^2 + eta0^2*(eta2)^2 + eta1^2*(eta2)^2 ) + 333.541311391*(eta0^2*eta1^2*eta2^2)'
    expression = '0.5*(1-tanh((bnds-0.75)*2*atanh(0.8)/0.11 ) )'
  []
  [multi_op_normal]
    type = ProjectionTensorMultiOP
    all_etas = 'eta0 eta1 eta2'
    projection_tensor_name = gb_normal
  []
  [aniso_tensor]
    type = GBDependentAnisotropicTensor
    gb = gb
    bulk_parameter = 0.1
    gb_parameter = 1.0 ##'${fparse ( 1 - 0.1) * (  5e-4 / ${GlobalParams/int_width} ) + 0.1 }'
    gb_normal_tensor_name = gb_normal
    gb_tensor_prop_name = aniso_tensor
  []
  [E0_Va_metal]
    type = ParsedMaterial
    material_property_names = 'T kB'
    constant_names = 'H0_f S0_f'
    constant_expressions = '1.56 3.3'
    expression = 'H0_f - S0_f*kB*T'
    property_name = 'E0_Va_metal'
  []
  [f_v]
    type = DerivativeParsedMaterial
    material_property_names = 'E0_Va_metal kB T V_a'
    property_name = f_v
    coupled_variables = 'c_v mu s11_aux s22_aux'
    expression = 'c_v*E0_Va_metal + kB*T*(c_v*log(c_v) + (1-c_v)*log(1-c_v) )'
  []
  [mu_v_prop]
    type = DerivativeParsedMaterial
    material_property_names = 'f_v(c_v) V_a df_dc:=D[f_v,c_v] mu_mech'
    property_name = mu_v_prop
    coupled_variables = 'c_v mu s11_aux s22_aux'
    expression = 'df_dc + V_a* mu_mech'
  []
  [mechanical_potential]
    type = StressBasedChemicalPotential
    property_name = mu_mech
    stress_name = stress
    direction_tensor_name = aniso_tensor
    prefactor_name = -1
  []
  [var_dependence]
    type = DerivativeParsedMaterial
    block = 0
    material_property_names = 'kB T'
    expression = 'c_v*(1.0-c_v)/kB/T'
    coupled_variables = c_v
    property_name = chi_v
    derivative_order = 1
  []
  [mobility]
    type = CompositeMobilityTensor
    block = 0
    M_name = M_v
    tensors = diffusivity
    weights = chi_v
    coupled_variables = 'c_v mu'
  []
  [diffusivity]
    type = GBDependentDiffusivity
    gb = gb
    bulk_parameter = 0.1
    gb_parameter = 10 ##'${fparse ( 1e4 - 0.1) * (  0.5 / ${GlobalParams/int_width} ) + 0.1 }'
    gb_normal_tensor_name = gb_normal
    gb_tensor_prop_name = diffusivity
  []
  [elasticity]
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = '${fparse 207 * 10^9 * 6.242e-9}'
    poissons_ratio = 0.3
  []
  [diffuse_strain_increment]
    type = FluxBasedStrainIncrement
    xflux = jx
    yflux = jy
    gb = gb
    property_name = diffuse
  []
  [eigenstrain_prefactor]
    type = DerivativeParsedMaterial
    property_name = 'eigenstrain_prefactor'
    coupled_variables = 'c_v mu'
    expression = '(c_v-2.2877916582962764e-7)'
  []
  [eigenstrain]
    type = ComputeVariableBaseEigenStrain
    base_tensor_property_name = aniso_tensor
    prefactor = eigenstrain_prefactor
    eigenstrain_name = eigenstrain
  []
  [gb_relax_prefactor]
    type = DerivativeParsedMaterial
    block = 0
    expression = '-R_gb*(c_v-2.2877916582962764e-7)*gb'
    coupled_variables = 'c_v gb'
    property_name = gb_relax_prefactor
    material_property_names = 'R_gb'
    derivative_order = 1
  []
  [gb_relax]
    type = GBRelaxationStrainIncrement
    property_name = gb_relax
    prefactor_name = gb_relax_prefactor
    gb_normal_name = gb_normal
  []
  [diffuse_creep_strain]
    type = SumTensorIncrements
    tensor_name = creep_strain
    coupled_tensor_increment_names = 'diffuse gb_relax'
  []
  [strain]
    type = ComputeIncrementalSmallStrain
    displacements = 'disp_x disp_y'
    eigenstrain_names = eigenstrain
  []
  [stress]
    type = ComputeStrainIncrementBasedStress
    inelastic_strain_names = creep_strain
  []
[]
[Preconditioning]
  [SMP]
    type = SMP
    full = true
  []
[]
[Executioner]
  type = Transient
  scheme = bdf2
  solve_type = NEWTON
  petsc_options_iname = '-pc_type -pc_factor_mat_solver_package'
  petsc_options_value = 'lu superlu_dist'
  l_tol = 1e-5
  l_max_its = 100
  nl_max_its = 12
  nl_abs_tol = 1e-9
  end_time = 1e5
  dt = 0.1

  [Adaptivity]
    max_h_level = 2
    refine_fraction = 0.3
    coarsen_fraction = 0.2
  []
  [TimeStepper]
    type = IterationAdaptiveDT
    dt = 1e-9
    iteration_window = 2
    optimal_iterations = 9
    growth_factor = 1.25
    cutback_factor = 0.8
  []
  dtmax = 1e3
[]
# [Debug]
#   show_var_residual_norms = true
# []
[Postprocessors]
  [total_cv]
    type = ElementIntegralVariablePostprocessor
    variable = c_v
  []
  [force_y]
    type = SideIntegralVariablePostprocessor
    variable = 's22_aux'
    boundary = top
  []
  [dy]
    type = SideAverageValue
    variable = 'disp_y'
    boundary = 'top'
    execute_on = 'INITIAL NONLINEAR TIMESTEP_END'
  []
[]
[Outputs]
  exodus = true
  perf_graph = true
  csv = true
  file_base = '${raw ${GlobalParams/file_base_name} ${fparse int ( ${GlobalParams/int_width} ) } }'
[]
