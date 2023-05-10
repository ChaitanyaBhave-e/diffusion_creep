[Mesh]
    type = GeneratedMesh
    dim = 2
    nx = '${fparse int ( 2 * xmax / ${GlobalParams/int_width} ) }'
    ny = '${fparse int ( 2 * ymax / ${GlobalParams/int_width} ) }'
    xmin = 0
    xmax = 100
    ymin = 0
    ymax = 100
    uniform_refine = 2
    skip_partitioning = true
  []
  
[GlobalParams]
    int_width = 10
    derivative_order = 2
    c_v_eq = 1.3563e-09
[]

[Variables]
    [c_v]
      initial_condition = ${GlobalParams/c_v_eq}
    []
    [c_cr]
      initial_condition = 0.18
    []
    [c_ni]
      initial_condition = 0.08
    []
[]
  
[AuxVariables]
    [eta]
    []
    [eta1]
    []
[]
  
[BCs]
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
  
[Kernels]
  #Vacancy diffusion
  [dc_v_dt]
    type = TimeDerivative
    variable = c_v
  []
  [./diff_v_v]
    type = MatDiffusion
    variable = c_v
    diffusivity = effD_v_v
  [../]
  [./diff_v_ni]
    type = MatDiffusion
    variable = c_v
    v = c_ni
    diffusivity = effD_v_ni
  [../]
  [./diff_v_cr]
    type = MatDiffusion
    variable = c_v
    v = c_cr
    diffusivity = effD_v_cr
  [../]
  
  [./source_v] 
    type = MaskedBodyForce
    variable = c_v
    value = 1e-6
    mask = bulk_mat
    coupled_variables = 'eta'
  [../]

  [GB_sink]
    type = MaskedBodyForce 
    variable = c_v
    mask = 'gb_sink_func'
    coupled_variables = 'eta'
  []

  [./c_ni_dot]
    type = TimeDerivative
    variable = c_ni
  [../]
  [./diff_ni_ni]
    type = MatDiffusion
    variable = c_ni
    diffusivity = effD_ni_ni
  [../]
  [./diff_ni_cr]
    type = MatDiffusion
    variable = c_ni
    v = c_cr
    diffusivity = effD_ni_cr
  [../]
  [./diff_ni_v]
    type = MatDiffusion
    variable = c_ni
    v = c_v
    diffusivity = effD_ni_v
  [../]
  
  [./c_cr_dot]
    type = TimeDerivative
    variable = c_cr
  [../]
  [./diff_cr_cr]
    type = MatDiffusion
    variable = c_cr
    diffusivity = effD_cr_cr
  [../]
  [./diff_cr_ni]
    type = MatDiffusion
    variable = c_cr
    v = c_ni
    diffusivity = effD_cr_ni
  [../]
  [./diff_cr_v]
    type = MatDiffusion
    variable = c_cr
    v = c_v
    diffusivity = effD_cr_v
  [../]
[]
  
[Materials]
    [gb_mat]
        type = DerivativeParsedMaterial
        coupled_variables = eta
        property_name = 'gb_mat'
        expression = '16*eta^2*(1-eta)^2'
    []
    [gb_sink_mat]
        type = DerivativeParsedMaterial
        coupled_variables = eta
        property_name = 'gb_sink_mat'
        expression = '(1.52e-5)^-1 * eta^8*(1.0-eta)^8'
    []
    [bulk_mat]
        type = DerivativeParsedMaterial
        coupled_variables = eta
        property_name = bulk_mat
        material_property_names = 'gb_mat(eta)'
        expression = '1-gb_mat'
    []

    [gb_sink_func]
        type = DerivativeParsedMaterial
        expression = '-R_gb*(c_v-${GlobalParams/c_v_eq})*gb_sink_mat'
        coupled_variables = 'c_v'
        property_name = gb_sink_func
        material_property_names = 'R_gb gb_sink_mat'
        derivative_order = 1
    []

    [./constants]
      type = GenericConstantMaterial
      prop_names =  'kB           T      c0_ni  c0_cr  theta0_ni_ni   theta0_ni_cr   theta0_cr_cr  R_gb'
      prop_values = '8.6173324e-5 1023   0.8    0.18   0.9884          1.386         1.052          1e8'
    [../]
  
    [./theta_ni_ni]
      type = DerivativeParsedMaterial
      coupled_variables = 'c_ni c_cr'
      property_name = theta_ni_ni
      material_property_names = 'kB T theta0_ni_ni'
      expression = 'theta0_ni_ni*(1-c_cr)/(1-c_ni-c_cr)/c_ni'
    [../]
    [./theta_ni_cr]
      type = DerivativeParsedMaterial
      coupled_variables = 'c_ni c_cr'
      property_name = theta_ni_cr
      material_property_names = 'kB T theta0_ni_cr'
      expression = 'theta0_ni_cr/(1-c_ni-c_cr)'
    [../]
    [./theta_cr_cr]
      type = DerivativeParsedMaterial
      coupled_variables = 'c_ni c_cr'
      property_name = theta_cr_cr
      material_property_names = 'kB T theta0_cr_cr'
      expression = 'theta0_cr_cr*(1-c_ni)/(1-c_ni-c_cr)/c_cr'
    [../]
    [./theta_v_v]
      type = DerivativeParsedMaterial
      coupled_variables = 'c_v'
      property_name = theta_v_v
      expression = '1/c_v'
    [../]

  # vacancy diff
  [M_v_v]
    type = DerivativeParsedMaterial
    property_name = M_v_v
    coupled_variables = 'c_v'
    expression = '(4.68e+8*c_v)' # nm^2/s
  []
  [M_v_cr]
    type = DerivativeParsedMaterial
    property_name = M_v_cr
    coupled_variables = 'c_v'
    expression = '-(1.749e+6*c_v)' # nm^2/s
  []
  [M_v_ni]
    type = DerivativeParsedMaterial
    property_name = M_v_ni
    coupled_variables = 'c_v'
    expression = '(1.977e+6*c_v)' # nm^2/s
  []

  # Cr diff
  [M_cr_cr]
    type = DerivativeParsedMaterial
    property_name = M_cr_cr
    coupled_variables = 'c_v'
    expression = '(5.518e+7*c_v)' # nm^2/s
  []
  [M_cr_v]
    type = DerivativeParsedMaterial
    property_name = M_cr_v
    coupled_variables = 'c_v'
    expression = '-(8.6e+7*c_v)' # nm^2/s
  []
  [M_cr_ni]
    type = DerivativeParsedMaterial
    property_name = M_cr_ni
    coupled_variables = 'c_v'
    expression = '-(5.384e+7*c_v)' # nm^2/s
  []

  # Ni diff
  [M_ni_ni]
    type = DerivativeParsedMaterial
    property_name = M_ni_ni
    coupled_variables = 'c_v'
    expression = '(5.787e+7*c_v)' # nm^2/s
  []
  [M_ni_v]
    type = DerivativeParsedMaterial
    property_name = M_ni_v
    coupled_variables = 'c_v'
    expression = '-(3.724e+8*c_v)' # nm^2/s
  []
  [M_ni_cr]
    type = DerivativeParsedMaterial
    property_name = M_ni_cr
    coupled_variables = 'c_v'
    expression = '-(5.208e+7*c_v)' # nm^2/s
  []
    
    # Effective diffusivity (Fe reference)
      [./effD_ni_ni]
        type = DerivativeParsedMaterial
        coupled_variables = 'c_v'
        property_name = effD_ni_ni
        material_property_names = 'M_ni_ni M_ni_cr theta_ni_ni theta_ni_cr'
        expression = '(M_ni_ni*theta_ni_ni + M_ni_cr*theta_ni_cr)'
      [../]
      [./effD_ni_cr] 
        type = DerivativeParsedMaterial
        coupled_variables = 'c_v' # eta'
        property_name = effD_ni_cr
        material_property_names = 'M_ni_ni M_ni_cr theta_cr_cr theta_ni_cr'
        expression = '(M_ni_ni*theta_ni_cr + M_ni_cr*theta_cr_cr)'
      [../]
      [./effD_ni_v]
        type = DerivativeParsedMaterial
        coupled_variables = 'c_v '
        property_name = effD_ni_v
        material_property_names = 'M_ni_v theta_v_v'
        expression = '(M_ni_v*theta_v_v)'
      [../]
    
      [./effD_cr_ni]
        type = DerivativeParsedMaterial
        coupled_variables = 'c_v' 
        property_name = effD_cr_ni
        material_property_names = 'M_cr_ni M_cr_cr theta_ni_ni theta_ni_cr'
        expression = '(M_cr_ni*theta_ni_ni + M_cr_cr*theta_ni_cr)'
      [../]
      [./effD_cr_cr]
        type = DerivativeParsedMaterial
        coupled_variables = 'c_v' 
        property_name = effD_cr_cr
        material_property_names = 'M_cr_ni M_cr_cr theta_ni_cr theta_cr_cr'
        expression = '(M_cr_ni*theta_ni_cr + M_cr_cr*theta_cr_cr)'
      [../]
      [./effD_cr_v]
        type = DerivativeParsedMaterial
        coupled_variables = 'c_v'
        property_name = effD_cr_v
        material_property_names = 'M_cr_v theta_v_v'
        expression = '(M_cr_v*theta_v_v)'
      [../]
    
      [./effD_v_ni]
        type = DerivativeParsedMaterial
        coupled_variables = 'c_v'
        property_name = effD_v_ni
        material_property_names = 'M_v_ni M_v_cr theta_ni_ni theta_ni_cr'
        expression = '(M_v_ni*theta_ni_ni + M_v_cr*theta_ni_cr)'
      [../]
      [./effD_v_cr]
        type = DerivativeParsedMaterial
        coupled_variables = 'c_v'
        property_name = effD_v_cr
        material_property_names = 'M_v_ni M_v_cr theta_ni_cr theta_cr_cr'
        expression = '(M_v_ni*theta_ni_cr + M_v_cr*theta_cr_cr)'
      [../]
      [./effD_v_v]
        type = DerivativeParsedMaterial
        coupled_variables = 'c_v'
        property_name = effD_v_v
        material_property_names = 'M_v_v theta_v_v'
        expression = '(M_v_v*theta_v_v)'
      [../]
[]
  
[Preconditioning]
  [./SMP]
    type = SMP
    full = true
  [../]
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
    
    end_time = 1e8
  
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
    # dtmax = 1e8
    # end_time = 100
[]
  
[Outputs]
  execute_on = 'TIMESTEP_END'
  exodus = true
  csv = true
  checkpoint = true
[]
  