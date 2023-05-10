[Mesh]
  type = GeneratedMesh
  dim = 2
  nx = '${fparse int ( 4 * xmax / ${GlobalParams/int_width} ) }'
  ny = '${fparse int ( 4 * ymax / ${GlobalParams/int_width} ) }'
  xmin = 0
  xmax = 1000
  ymin = 0
  ymax = 1000
  uniform_refine = 1
  skip_partitioning = true
[]

[GlobalParams]
  displacements = 'disp_x disp_y'
  derivative_order = 2
  int_width = 10 #0
  c_v_eq = 2.2877916582962764e-7
[]

[Variables]
  [mu]
  []
  [c_v]
    initial_condition = ${GlobalParams/c_v_eq}
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
  [eta]
  []
  [eta1]
  []
  [gb]
    order = CONSTANT
    family = MONOMIAL
  []

[]

[ICs]
  [eta_grain]
    type = BoundingBoxIC
    variable = eta
    x1 = ${Mesh/xmin}
    x2 = ${Mesh/xmax}
    y1 = ${Mesh/ymin}
    y2 = ${Mesh/ymax}
    inside = 1
    outside = 0
  []
  [eta1_grain]
    type = BoundingBoxIC
    variable = eta1
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

  [gb_c_v]
    type = DirichletBC
    variable = c_v
    value = ${GlobalParams/c_v_eq}
    boundary = 'top left bottom right'
  []

[]

[Functions]
  [pressure_func]
    type = ParsedFunction
    symbol_names = 'P0'
    symbol_values = '${fparse 50 * 10^6 * 6.242e-9}'
    expression = -P0
  []
[]

[Kernels]
  #Tensor mechanics
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
  [gb_val]
    type = MaterialRealAux
    variable = gb
    property = gb_mat
  []
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
[]

[Materials]
  [constants]
    type = GenericConstantMaterial
    prop_names = 'kB T V_a R'
    prop_values = '8.6173324e-5 973 0.01099207207 8.314462618'
  []
  [gb_mat]
    type = DerivativeParsedMaterial
    coupled_variables = eta
    property_name = 'gb_mat'
    expression = '16*eta^2*(1-eta)^2'
  []

  [multi_op_normal]
    type = ProjectionTensorMultiOP
    all_etas = 'eta eta1'
    projection_tensor_name = gb_normal
  []
  [aniso_tensor]
    type = GBDependentAnisotropicTensor
    gb = gb
    bulk_parameter = 0.1
    gb_parameter = 1
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
    # outputs = exodus
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
    outputs = exodus
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
  [M_v]
    type = CompositeMobilityTensor
    M_name = M_v
    tensors = diffusivity
    weights = chi_v
    coupled_variables = 'c_v mu'
  []
  [D_func]
    type = ParsedMaterial
    property_name = D
    material_property_names = 'R T gb_mat'
    expression = '1.9e14*exp(-66800*4.184/R/T)/${GlobalParams/c_v_eq}' #nm^2/s
    outputs = exodus
  []
  [diffusivity]
    type = IsotropicDiffusionTensor
    diffusion_coefficient = D
    diffusivity_tensor_name = diffusivity
  []
  [scalar_M]
    type = ParsedMaterial
    property_name = 'M_scalar'
    material_property_names = 'D chi_v'
    expression = 'D*chi_v'
    outputs = exodus
  []
  [elasticity]
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = '${fparse 207 * 10^9 * 6.242e-9}'
    poissons_ratio = 0.3
  []

  #Creep strain increments
  [diffuse_strain_increment]
    type = FluxBasedStrainIncrement
    xflux = jx
    yflux = jy
    gb = gb
    property_name = diffuse
  []
  [diffuse_creep_strain]
    type = SumTensorIncrements
    tensor_name = creep_strain
    coupled_tensor_increment_names = 'diffuse'
  []

  #Eigen strain
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
    # full = true
  []
[]

[Executioner]
  type = Transient
  scheme = bdf2
  solve_type = PJFNK
  # petsc_options_iname = '-pc_type'
  # petsc_options_value = 'lu'
  # petsc_options_iname = '-pc_type -pc_factor_mat_solver_package'
  # petsc_options_value = 'lu superlu_dist'
  # petsc_options_iname = '-pc_type -pc_hypre_type -ksp_gmres_restart -pc_hypre_boomeramg_strong_threshold'
  # petsc_options_value = 'hypre    boomeramg      31                 0.7'
  petsc_options_iname = '-pc_type -ksp_grmres_restart -sub_ksp_type -sub_pc_type -pc_asm_overlap'
  petsc_options_value = 'asm      31                  preonly       lu           2'
  l_tol = 1e-4 #1e-5
  l_max_its = 100
  nl_abs_tol = 1e-9
  end_time = 1e5
  dt = 0.1

  # [Adaptivity]
  #   max_h_level = 1
  #   refine_fraction = 0.3
  #   coarsen_fraction = 0.2
  # []

  [TimeStepper]
    type = IterationAdaptiveDT
    dt = 1e-9
    iteration_window = 2
    optimal_iterations = 9
    growth_factor = 1.25
    cutback_factor = 0.8
  []
  dtmax = 1e5
  # end_time = 100
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
  [creep_dy]
    type = ChangeOverTimePostprocessor
    postprocessor = 'dy'
    # change_with_respect_to_initial = true
    execute_on = 'INITIAL TIMESTEP_END'
  []
  [flux_x]
    type = SideDiffusiveFluxIntegral
    variable = mu
    diffusivity = 'M_scalar'
    boundary = 'left'

  []
[]

[Outputs]
  exodus = true
  perf_graph = true
  csv = true
[]
