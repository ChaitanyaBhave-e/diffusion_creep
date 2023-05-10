[Mesh] #Sets mesh size to 10 microns by 10 microns
  [gen]
    type = GeneratedMeshGenerator
    dim = 2
    nx = 150
    ny = 150
    xmin = -25
    xmax = 25
    ymin = -25
    ymax = 25
  []
[]

[GlobalParams]
  op_num = 3
  var_name_base = eta # base name of grains
  #   v = 'eta0 eta1 eta2' # Names of the grains

  theta1 = 120 # Angle the first grain makes at the triple junction
  theta2 = 120 # Angle the second grain makes at the triple junction
  junction = '0 0 0'
[]

[Variables]
  [eta0]
    [InitialCondition]
      type = TricrystalTripleJunctionIC
      op_index = 1
    []
  []
  [eta1]
    [InitialCondition]
      type = TricrystalTripleJunctionIC
      op_index = 2
    []
  []
  [eta2]
    [InitialCondition]
      type = TricrystalTripleJunctionIC
      op_index = 3
    []
  []
  [gb]
  []
[]

[AuxVariables]
  [bnds]
  []
[]

[Kernels]
  #GB value
  [gb]
    type = MaterialPropertyValue
    variable = gb
    prop_name = gb_mat
  []

  [PolycrystalKernel]
    # Custom action creating all necessary kernels for grain growth.  All input parameters are up in GlobalParams
  []
[]

[AuxKernels]
  [bnds_aux]
    type = BndsCalcAux
    variable = bnds
    v = 'eta0 eta1 eta2'
  []
[]

[Materials]
  [gb_mat]
    type = DerivativeParsedMaterial
    coupled_variables = 'eta0 eta1 eta2 bnds'
    property_name = 'gb_mat'
    expression = '16*(eta0^2*eta1^2 + eta1^2*eta2^2 + eta0^2*eta2^2)' #' + (2/3)*0.43075792*3^6*(eta0^2*eta1^2*eta2^2)'
    # expression = '0.5*(1-tanh((bnds-0.75)*2*atanh(0.8)/0.11 ) )'
    # expression = 'exp(-(bnds-0.5)/0.01)'
  []
  [gb_norm]
    type = PhaseNormalTensor
    normal_tensor_name = 'gb_normal'
    phase = bnds
    outputs = exodus
  []
  [multiop_norm]
    type = ProjectionTensorMultiOP
    projection_tensor_name = 'multiop_norm'
    all_etas = 'eta0 eta1 eta2'
    outputs = exodus
  []
  # [eta2_norm]
  #   type = PhaseNormalTensor
  #   normal_tensor_name = 'eta2_norm'
  #   phase = eta2
  #   outputs = exodus
  # []

  [eta0_eta1_norm]
    type = ProjectionTensor2OP
    vi = eta0
    vj = eta1
    projection_tensor_name = 'eta0_eta1_norm'
    outputs = exodus
  []
  # [eta1_eta2_norm]
  #   type = ProjectionTensor2OP
  #   vi = eta1
  #   vj = eta2
  #   projection_tensor_name = 'eta1_eta2_norm'
  #   outputs = exodus
  # []
  # [eta0_eta2_norm]
  #   type = ProjectionTensor2OP
  #   vi = eta0
  #   vj = eta2
  #   projection_tensor_name = 'eta0_eta2_norm'
  #   outputs = exodus
  # []

  [material]
    # Material properties
    type = GBEvolution
    T = 450 # Constant temperature of the simulation (for mobility calculation)
    wGB = 2.5 # Width of the diffuse GB
    GBmob0 = 2.5e-6 #m^4(Js) for copper from Schoenfelder1997
    Q = 0.23 #eV for copper from Schoenfelder1997
    GBenergy = 0.708 #J/m^2 from Schoenfelder1997
  []
[]

[Executioner]
  type = Transient
  scheme = bdf2
  solve_type = NEWTON #PJFNK
  petsc_options_iname = '-pc_type'
  petsc_options_value = 'lu'
  l_tol = 1e-5
  l_max_its = 100 #00
  nl_abs_tol = 1e-9
  end_time = 1.0
  dt = 0.1
  dtmax = 0.5
  [TimeStepper]
    type = IterationAdaptiveDT
    dt = 1e-2
    iteration_window = 2
    optimal_iterations = 9
    growth_factor = 1.25
    cutback_factor = 0.8
  []
  num_steps = 1
[]

[Outputs]
  exodus = true
  perf_graph = true
  # csv = true
[]
