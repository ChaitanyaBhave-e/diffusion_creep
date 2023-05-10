[Mesh]
  type = GeneratedMesh
  dim = 2
  nx = '${fparse int ( 4 * xmax / ${GlobalParams/int_width} ) }'
  ny = '${fparse int ( 4 * ymax / ${GlobalParams/int_width} ) }'
  xmin = 0
  xmax = 100 #0
  ymin = 0
  ymax = 100 #0
  uniform_refine = 1
  # skip_partitioning = true
[]

[GlobalParams]
  displacements = 'disp_x disp_y'
  derivative_order = 2
  int_width = 10
  c_v_eq = 1.3563e-09
[]

[Variables]
  [mu_v]
  []
  [c_v]
    initial_condition = ${GlobalParams/c_v_eq}
  []
  [c_cr]
    initial_condition = 0.18
  []
  [c_ni]
    initial_condition = 0.08
  []
  [mu_cr]
  []
  [mu_ni]
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
  [jx_vv]
    family = MONOMIAL
  []
  [jy_vv]
    family = MONOMIAL
  []
  [jx_vcr]
    family = MONOMIAL
  []
  [jy_vcr]
    family = MONOMIAL
  []
  [jx_vni]
    family = MONOMIAL
  []
  [jy_vni]
    family = MONOMIAL
  []
  [jx_v]
    family = MONOMIAL
  []
  [jy_v]
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

  # [gb_c_v]
  #   type = DirichletBC
  #   variable = c_v
  #   value = ${GlobalParams/c_v_eq} # 1.3563e-09
  #   boundary = 'top left bottom right'
  # []

[]

[Functions]
  [pressure_func]
    type = ParsedFunction
    symbol_names = 'P0'
    symbol_values = '${fparse 200 * 10^6 * 6.242e-9}'
    expression = -P0
  []
[]

[AuxKernels] 
  [flux_x_vv]
    type = DiffusionFluxAux
    variable = jx_vv
    diffusion_variable = mu_v
    diffusivity = M_v_v
    component = x
  []
  [flux_y_vv]
    type = DiffusionFluxAux
    variable = jy_vv
    diffusion_variable = mu_v
    diffusivity = M_v_v
    component = y
  []
  [flux_x_vcr]
    type = DiffusionFluxAux
    variable = jx_vcr
    diffusion_variable = mu_cr
    diffusivity = M_v_cr
    component = x
  []
  [flux_y_vcr]
    type = DiffusionFluxAux
    variable = jy_vcr
    diffusion_variable = mu_cr
    diffusivity = M_v_cr
    component = y
  []
  [flux_x_vni]
    type = DiffusionFluxAux
    variable = jx_vni
    diffusion_variable = mu_ni
    diffusivity = M_v_ni
    component = x
  []
  [flux_y_vni]
    type = DiffusionFluxAux
    variable = jy_vni
    diffusion_variable = mu_ni
    diffusivity = M_v_ni
    component = y
  []
  [flux_x_v]
    type = ParsedAux
    variable = jx_v
    coupled_variables = 'jx_vv jx_vcr jx_vni'
    expression = 'jx_vv+jx_vcr+jx_vni'
  []
  [flux_y_v]
    type = ParsedAux
    variable = jy_v
    coupled_variables = 'jy_vv jy_vcr jy_vni'
    expression = 'jy_vv+jy_vcr+jy_vni'
  []
[]

[Kernels]
  #Tensor mechanics
  [TensorMechanics]
    displacements = 'disp_x disp_y'
  []

  #Vacancy diffusion
  [dc_v_dt]
    type = TimeDerivative
    variable = c_v
  []
  [chempot_v]
    type = CHSplitChemicalPotential
    variable = mu_v
    chemical_potential_prop = mu_v_prop
    c = c_v
  []
  [diff_v_v]
    type = SplitCHWRes
    variable = c_v
    mob_name = M_v_v
    w = mu_v
  []
  [diff_v_cr]
    type = SplitCHWRes
    variable = c_v
    mob_name = M_v_cr
    w = mu_cr
  []
  [diff_v_ni]
    type = SplitCHWRes
    variable = c_v
    mob_name = M_v_ni
    w = mu_ni
  []

  # Cr diffusion
  [dc_cr_dt]
    type = TimeDerivative
    variable = c_cr
  []
  [chempot_cr]
    type = CHSplitChemicalPotential
    variable = mu_cr
    chemical_potential_prop = mu_cr_prop
    c = c_cr
  []
  [diff_cr_cr]
    type = SplitCHWRes
    variable = c_cr
    mob_name = M_cr_cr
    w = mu_cr
  []
  [diff_cr_v]
    type = SplitCHWRes
    variable = c_cr
    mob_name = M_cr_v
    w = mu_v
  []
  [diff_cr_ni]
    type = SplitCHWRes
    variable = c_cr
    mob_name = M_cr_ni
    w = mu_ni
  []

  # Ni diffusion
  [dc_ni_dt]
    type = TimeDerivative
    variable = c_ni
  []
  [chempot_ni]
    type = CHSplitChemicalPotential
    variable = mu_ni
    chemical_potential_prop = mu_ni_prop
    c = c_ni
  []
  [diff_ni_ni]
    type = SplitCHWRes
    variable = c_ni
    mob_name = M_ni_ni
    w = mu_ni
  []
  [diff_ni_v]
    type = SplitCHWRes
    variable = c_ni
    mob_name = M_ni_v
    w = mu_v
  []
  [diff_ni_cr]
    type = SplitCHWRes
    variable = c_ni
    mob_name = M_ni_cr
    w = mu_cr
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
    prop_names =  'kB           T     V_a           R           c0_cr  c0_ni  ceq_v'
    prop_values = '8.6173324e-5 1023 0.01099207207 8.314462618  0.18   0.08 1.3563e-09' 
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
    gb_parameter = 0.1
    gb_normal_tensor_name = gb_normal
    gb_tensor_prop_name = aniso_tensor
  []

  [f_alloy] # Taylor FE
    type = DerivativeParsedMaterial
    property_name = f_alloy
    coupled_variables = 'c_cr c_ni'
    material_property_names = 'c0_ni c0_cr'
    constant_names =       'f0         mu0_ni     mu0_cr      theta0_nini      theta0_crcr        theta0_nicr   '
    constant_expressions = '-0.4944   -0.2707    0.02844      1.233            0.6355             0.1246        ' # eV 
    expression = 'f0 + mu0_ni*(c_ni-c0_ni) + mu0_cr*(c_cr-c0_cr) + 0.5*theta0_nini*(c_ni-c0_ni)^2 + 0.5*theta0_crcr*(c_cr-c0_cr)^2 + theta0_nicr*(c_ni-c0_ni)*(c_cr-c0_cr)'
  []
  [f_defect]
    type = DerivativeParsedMaterial
    property_name = f_defect
    coupled_variables = 'c_v'
    material_property_names = 'kB T'
    expression = 'kB*T*(c_v*log(c_v/${GlobalParams/c_v_eq}) + (1-c_v)*log(1-c_v))' # eV
  []

  [mu_v_prop] 
    type = DerivativeParsedMaterial
    material_property_names = 'f_defect(c_v) V_a df_dc_v:=D[f_defect,c_v] mu_v_mech'
    property_name = mu_v_prop
    coupled_variables = 'c_v mu_v s11_aux s22_aux'
    expression = 'df_dc_v + V_a* mu_v_mech' # eV
  []
  [mu_cr_prop]
    type = DerivativeParsedMaterial
    material_property_names = 'f_alloy(c_ni,c_cr) df_dc_cr:=D[f_alloy,c_cr]'
    property_name = mu_cr_prop
    coupled_variables = 'c_cr c_ni'
    expression = 'df_dc_cr'
  []
  [mu_ni_prop]
    type = DerivativeParsedMaterial
    material_property_names = 'f_alloy(c_ni,c_cr) df_dc_ni:=D[f_alloy,c_ni]'
    property_name = mu_ni_prop
    coupled_variables = 'c_cr c_ni'
    expression = 'df_dc_ni'
  []

  [mechanical_potential]
    type = StressBasedChemicalPotential
    property_name = mu_v_mech
    stress_name = stress
    direction_tensor_name = aniso_tensor
    prefactor_name = -1
    outputs = exodus
  []
 
   # vacancy diff
   [M_v_v]
    type = DerivativeParsedMaterial
    property_name = M_v_v
    coupled_variables = 'c_v'
    material_property_names = 'kB T'
    expression = '(4.68e+8*c_v)/kB/T' # nm^2/(eV.K.s)
  []
  [M_v_cr]
    type = DerivativeParsedMaterial
    property_name = M_v_cr
    coupled_variables = 'c_v'
    material_property_names = 'kB T'
    expression = '-(1.749e+6*c_v)/kB/T' # nm^2/(eV.K.s)
  []
  [M_v_ni]
    type = DerivativeParsedMaterial
    property_name = M_v_ni
    coupled_variables = 'c_v'
    material_property_names = 'kB T'
    expression = '(1.977e+6*c_v)/kB/T' # nm^2/(eV.K.s)
  []

  # Cr diff
  [M_cr_cr]
    type = DerivativeParsedMaterial
    property_name = M_cr_cr
    coupled_variables = 'c_v'
    material_property_names = 'kB T'
    expression = '(5.518e+7*c_v)/kB/T' # nm^2/(eV.K.s)
  []
  [M_cr_v]
    type = DerivativeParsedMaterial
    property_name = M_cr_v
    coupled_variables = 'c_v'
    material_property_names = 'kB T'
    expression = '-(8.6e+7*c_v)/kB/T' # nm^2/(eV.K.s)
  []
  [M_cr_ni]
    type = DerivativeParsedMaterial
    property_name = M_cr_ni
    coupled_variables = 'c_v'
    material_property_names = 'kB T'
    expression = '-(5.384e+7*c_v)/kB/T' # nm^2/(eV.K.s)
  []

  # Ni diff
  [M_ni_ni]
    type = DerivativeParsedMaterial
    property_name = M_ni_ni
    coupled_variables = 'c_v'
    material_property_names = 'kB T'
    expression = '(5.787e+7*c_v)/kB/T' # nm^2/(eV.K.s)
  []
  [M_ni_v]
    type = DerivativeParsedMaterial
    property_name = M_ni_v
    coupled_variables = 'c_v'
    material_property_names = 'kB T'
    expression = '-(3.724e+8*c_v)/kB/T' # nm^2/(eV.K.s)
  []
  [M_ni_cr]
    type = DerivativeParsedMaterial
    property_name = M_ni_cr
    coupled_variables = 'c_v'
    material_property_names = 'kB T'
    expression = '-(5.208e+7*c_v)/kB/T' # nm^2/(eV.K.s)
  []

  [elasticity]
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = '${fparse 207 * 10^9 * 6.242e-9}'
    poissons_ratio = 0.3
  []

  [diffuse_strain_increment]
    type = FluxBasedStrainIncrement
    xflux = jx_v
    yflux = jy_v
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
    coupled_variables = 'c_v mu_v'
    expression = '(c_v-${GlobalParams/c_v_eq})'
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
    full = true
  []
[]

[Executioner]
  type = Transient
  scheme = bdf2
  solve_type = NEWTON
  # petsc_options_iname = '-pc_type'
  # petsc_options_value = 'lu'
  petsc_options_iname = '-pc_type -pc_factor_mat_solver_package'
  petsc_options_value = 'lu superlu_dist'
  # petsc_options_iname = '-pc_type -pc_hypre_type -ksp_gmres_restart -pc_hypre_boomeramg_strong_threshold'
  # petsc_options_value = 'hypre    boomeramg      31                 0.7'
  # petsc_options_iname = '-pc_type -ksp_grmres_restart -sub_ksp_type -sub_pc_type -pc_asm_overlap'
  # petsc_options_value = 'asm      31                  preonly       lu           2'
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
    dt = 1e-6
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
[]

[Outputs]
  exodus = true
  perf_graph = true
  csv = true
[]
