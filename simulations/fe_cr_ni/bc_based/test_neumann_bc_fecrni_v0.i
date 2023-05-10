[Mesh]
  type = GeneratedMesh
  dim = 2
  nx = '${fparse int ( 2 * xmax / ${GlobalParams/int_width} ) }'
  ny = '${fparse int ( 2 * ymax / ${GlobalParams/int_width} ) }'
  xmin = 0
  xmax = 50
  ymin = 0
  ymax = 50
  uniform_refine = 0
  skip_partitioning = true
[]

[GlobalParams]
  displacements = 'disp_x disp_y'
  derivative_order = 2
  int_width = 4
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

  [jx_vv]
  []
  [jy_vv]
  []
  [jx_vcr]
  []
  [jy_vcr]
  []
  [jx_vni]
  []
  [jy_vni]
  []

  [jx_crcr]
  []
  [jy_crcr]
  []
  [jx_crv]
  []
  [jy_crv]
  []
  [jx_crni]
  []
  [jy_crni]
  []

  [jx_nini]
  []
  [jy_nini]
  []
  [jx_niv]
  []
  [jy_niv]
  []
  [jx_nicr]
  []
  [jy_nicr]
  []
[]

[AuxVariables]
  [s11_aux]
    order = CONSTANT
    family = MONOMIAL
  []
  [s12_aux]
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
  [s0_aux]
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
  [jx_v]
    order = FIRST
    family = MONOMIAL
  []
  [jy_v]
    order = FIRST
    family = MONOMIAL
  []
[]

[ICs]
  [eta_grain]
    type = BoundingBoxIC
    variable = eta
    x1 = -${Mesh/xmax}
    x2 = ${Mesh/xmax}
    y1 = -${Mesh/ymax}
    y2 = ${Mesh/ymax}
    inside = 1
    outside = 0
  []
  [eta1_grain]
    type = BoundingBoxIC
    variable = eta1
    x1 = -${Mesh/xmax}
    x2 = ${Mesh/xmax}
    y1 = -${Mesh/ymax}
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
  [left_x]
    type = DirichletBC
    variable = disp_x
    boundary = left
    value = 0
  []

  [sink_top_y]
    type = MatNeumannBC
    variable = c_v
    boundary = 'top'
    value = 1
    boundary_material = neumann_sink_top_y
  []
  [sink_right_x]
    type = MatNeumannBC
    variable = c_v
    boundary = 'right'
    value = 1
    boundary_material = neumann_sink_right_x
  []

  [gb_c_v]
    type = DirichletBC
    variable = c_v
    value = ${GlobalParams/c_v_eq} # 1.3563e-09
    boundary = 'top right'
  []
[]

[Kernels]
  # Tensor mechanics
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

  [flux_x_vv]
    type = CHSplitFlux
    variable = jx_vv
    component = '0'
    mobility_name = tensor_M_v_v
    mu = mu_v
    c = c_v
  []
  [flux_y_vv]
    type = CHSplitFlux
    variable = jy_vv
    component = '1'
    mobility_name = tensor_M_v_v
    mu = mu_v
    c = c_v
  []
  [flux_x_vcr]
    type = CHSplitFlux
    variable = jx_vcr
    component = '0'
    mobility_name = tensor_M_v_cr
    mu = mu_cr
    c = c_v
  []
  [flux_y_vcr]
    type = CHSplitFlux
    variable = jy_vcr
    component = '1'
    mobility_name = tensor_M_v_cr
    mu = mu_cr
    c = c_v
  []
  [flux_x_vni]
    type = CHSplitFlux
    variable = jx_vni
    component = '0'
    mobility_name = tensor_M_v_ni
    mu = mu_ni
    c = c_v
  []
  [flux_y_vni]
    type = CHSplitFlux
    variable = jy_vni
    component = '1'
    mobility_name = tensor_M_v_ni
    mu = mu_ni
    c = c_v
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

  [flux_x_crcr]
    type = CHSplitFlux
    variable = jx_crcr
    component = '0'
    mobility_name = tensor_M_cr_cr
    mu = mu_cr
    c = c_cr
  []
  [flux_y_crcr]
    type = CHSplitFlux
    variable = jy_crcr
    component = '1'
    mobility_name = tensor_M_cr_cr
    mu = mu_v
    c = c_cr
  []
  [flux_x_crv]
    type = CHSplitFlux
    variable = jx_crv
    component = '0'
    mobility_name = tensor_M_cr_v
    mu = mu_v
    c = c_cr
  []
  [flux_y_crv]
    type = CHSplitFlux
    variable = jy_crv
    component = '1'
    mobility_name = tensor_M_cr_v
    mu = mu_v
    c = c_cr
  []
  [flux_x_crni]
    type = CHSplitFlux
    variable = jx_crni
    component = '0'
    mobility_name = tensor_M_cr_ni
    mu = mu_ni
    c = c_cr
  []
  [flux_y_crni]
    type = CHSplitFlux
    variable = jy_crni
    component = '1'
    mobility_name = tensor_M_cr_ni
    mu = mu_ni
    c = c_cr
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

  [flux_x_nini]
    type = CHSplitFlux
    variable = jx_nini
    component = '0'
    mobility_name = tensor_M_ni_ni
    mu = mu_ni
    c = c_ni
  []
  [flux_y_nini]
    type = CHSplitFlux
    variable = jy_nini
    component = '1'
    mobility_name = tensor_M_ni_ni
    mu = mu_v
    c = c_ni
  []
  [flux_x_niv]
    type = CHSplitFlux
    variable = jx_niv
    component = '0'
    mobility_name = tensor_M_ni_v
    mu = mu_v
    c = c_ni
  []
  [flux_y_niv]
    type = CHSplitFlux
    variable = jy_niv
    component = '1'
    mobility_name = tensor_M_ni_v
    mu = mu_v
    c = c_ni
  []
  [flux_x_nicr]
    type = CHSplitFlux
    variable = jx_nicr
    component = '0'
    mobility_name = tensor_M_ni_cr
    mu = mu_cr
    c = c_ni
  []
  [flux_y_nicr]
    type = CHSplitFlux
    variable = jy_nicr
    component = '1'
    mobility_name = tensor_M_ni_cr
    mu = mu_cr
    c = c_ni
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
  [matl_s12]
    type = RankTwoAux
    rank_two_tensor = stress
    index_i = 0
    index_j = 1
    variable = s12_aux
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
  [matl_s0]
    type = ParsedAux
    variable = s0_aux
    coupled_variables = 's11_aux s22_aux s33_aux'
    expression = 's11_aux+s22_aux+s33_aux'
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

[Materials]
  [constants]
    type = GenericConstantMaterial  # V_a is for NiCr
    prop_names =  'kB           T     V_a           R           c0_cr  c0_ni  ceq_v'
    prop_values = '8.6173324e-5 1023 0.01099207207 8.314462618  0.18   0.08 1.3563e-09' 
  []
  [gb_mat]
    type = DerivativeParsedMaterial
    coupled_variables = eta
    property_name = 'gb_mat'
    expression = '16*eta^2*(1-eta)^2'
  []

  [s11_mat]
    type = ParsedMaterial
    coupled_variables = 's11_aux'
    property_name = s11_mat
    expression = 's11_aux'
  []
  [s12_mat]
    type = ParsedMaterial
    coupled_variables = 's12_aux'
    property_name = s12_mat
    expression = 's12_aux'
  []
  [s22_mat]
    type = ParsedMaterial
    coupled_variables = 's22_aux'
    property_name = s22_mat
    expression = 's22_aux'
  []

  [traction_vector]
    type = TractionVector
    tx_name = tx
    ty_name = ty
    etai = eta
    outputs = exodus
  []

  [neumann_sink_top_y]
    type = ParsedMaterial
    expression = '1e-4' 
    property_name = neumann_sink_top_y
    outputs = exodus
  []
  [neumann_sink_right_x]
    type = ParsedMaterial
    expression = '-1e-4' 
    property_name = neumann_sink_right_x
    outputs = exodus
  []

  [c_v_norm]
    type = DerivativeParsedMaterial
    property_name = c_v_norm
    coupled_variables = 'c_v'
    expression = 'c_v/${GlobalParams/c_v_eq}'
    output_properties = 'c_v_norm'
    outputs = exodus
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
    expression = 'kB*T*(c_v*log(c_v/${GlobalParams/c_v_eq}))' # eV
  []

  [mu_v_prop] 
    type = DerivativeParsedMaterial
    material_property_names = 'f_defect(c_v) V_a df_dc_v:=D[f_defect,c_v]'
    property_name = mu_v_prop
    coupled_variables = 'c_v mu_v s11_aux s22_aux'
    expression = 'df_dc_v' # eV
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

 # corrected diffusivities for 18Cr8Ni
 # vacancy diff
 [M_v_v]
  type = DerivativeParsedMaterial
  property_name = M_v_v
  coupled_variables = 'c_v'
  material_property_names = 'kB T'
  expression = '(1.794e+8*c_v)/kB/T' # nm^2/(eV.K.s)
[]
[M_v_cr]
  type = DerivativeParsedMaterial
  property_name = M_v_cr
  coupled_variables = 'c_v'
  material_property_names = 'kB T'
  expression = '-(1.543e+7*c_v)/kB/T' # nm^2/(eV.K.s)
[]
[M_v_ni]
  type = DerivativeParsedMaterial
  property_name = M_v_ni
  coupled_variables = 'c_v'
  material_property_names = 'kB T'
  expression = '(6.903e+6*c_v)/kB/T' # nm^2/(eV.K.s)
[]

# Cr diff
[M_cr_cr]
  type = DerivativeParsedMaterial
  property_name = M_cr_cr
  coupled_variables = 'c_v'
  material_property_names = 'kB T'
  expression = '(3.147e+7*c_v)/kB/T' # nm^2/(eV.K.s)
[]
[M_cr_v]
  type = DerivativeParsedMaterial
  property_name = M_cr_v
  coupled_variables = 'c_v'
  material_property_names = 'kB T'
  expression = '-(4.772e+7*c_v)/kB/T' # nm^2/(eV.K.s)
[]
[M_cr_ni]
  type = DerivativeParsedMaterial
  property_name = M_cr_ni
  coupled_variables = 'c_v'
  material_property_names = 'kB T'
  expression = '-(3.384e+6*c_v)/kB/T' # nm^2/(eV.K.s)
[]

# Ni diff
[M_ni_ni]
  type = DerivativeParsedMaterial
  property_name = M_ni_ni
  coupled_variables = 'c_v'
  material_property_names = 'kB T'
  expression = '(5.29e+6*c_v)/kB/T' # nm^2/(eV.K.s)
[]
[M_ni_v]
  type = DerivativeParsedMaterial
  property_name = M_ni_v
  coupled_variables = 'c_v'
  material_property_names = 'kB T'
  expression = '-(7.446e+6*c_v)/kB/T' # nm^2/(eV.K.s)
[]
[M_ni_cr]
  type = DerivativeParsedMaterial
  property_name = M_ni_cr
  coupled_variables = 'c_v'
  material_property_names = 'kB T'
  expression = '-(9.073e+5*c_v)/kB/T' # nm^2/(eV.K.s)
[]

  [mobility_v_v]
    type = IsotropicDiffusionTensor
    diffusion_coefficient = M_v_v
    diffusivity_tensor_name = tensor_M_v_v
  []
  [mobility_v_cr]
    type = IsotropicDiffusionTensor
    diffusion_coefficient = M_v_cr
    diffusivity_tensor_name = tensor_M_v_cr
  []
  [mobility_v_ni]
    type = IsotropicDiffusionTensor
    diffusion_coefficient = M_v_ni
    diffusivity_tensor_name = tensor_M_v_ni
  []

  [mobility_cr_cr]
    type = IsotropicDiffusionTensor
    diffusion_coefficient = M_cr_cr
    diffusivity_tensor_name = tensor_M_cr_cr
  []
  [mobility_cr_v]
    type = IsotropicDiffusionTensor
    diffusion_coefficient = M_cr_v
    diffusivity_tensor_name = tensor_M_cr_v
  []
  [mobility_cr_ni]
    type = IsotropicDiffusionTensor
    diffusion_coefficient = M_cr_ni
    diffusivity_tensor_name = tensor_M_cr_ni
  []

  [mobility_ni_ni]
    type = IsotropicDiffusionTensor
    diffusion_coefficient = M_ni_ni
    diffusivity_tensor_name = tensor_M_ni_ni
  []
  [mobility_ni_v]
    type = IsotropicDiffusionTensor
    diffusion_coefficient = M_ni_v
    diffusivity_tensor_name = tensor_M_ni_v
  []
  [mobility_ni_cr]
    type = IsotropicDiffusionTensor
    diffusion_coefficient = M_ni_cr
    diffusivity_tensor_name = tensor_M_ni_cr
  []

  [elasticity] # This is for Ni
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = '${fparse 207 * 10^9 * 6.242e-9}'
    poissons_ratio = 0.3
  []

  #Creep strain increments
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

  [strain]
    type = ComputeIncrementalSmallStrain
    displacements = 'disp_x disp_y'
    # eigenstrain_names = eigenstrain
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
  petsc_options_iname = '-pc_type'
  petsc_options_value = 'lu'
  # petsc_options_iname = '-pc_type -pc_factor_mat_solver_package'
  # petsc_options_value = 'lu superlu_dist'
  # petsc_options_iname = '-pc_type -pc_hypre_type -ksp_gmres_restart -pc_hypre_boomeramg_strong_threshold'
  # petsc_options_value = 'hypre    boomeramg      31                 0.7'
  # petsc_options_iname = '-pc_type -ksp_grmres_restart -sub_ksp_type -sub_pc_type -pc_asm_overlap'
  # petsc_options_value = 'asm      31                  preonly       lu           2'
  l_tol = 1e-4 #1e-3
  l_max_its = 100
  nl_abs_tol = 1e-9
  end_time = 1e5
  dt = 0.1

  # [Adaptivity]
  #   max_h_level = 2
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
  checkpoint = true
[]
