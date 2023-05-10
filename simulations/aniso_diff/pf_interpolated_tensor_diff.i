# This example calculates the effective thermal conductivity across a microstructure
# with circular second phase precipitates. Two methods are used to calculate the effective thermal conductivity,
# the direct method that applies a temperature to one side and a heat flux to the other,
# and the AEH method.
[Mesh] #Sets mesh size to 10 microns by 10 microns
  [gen]
    type = GeneratedMeshGenerator
    dim = 2
    nx = 200
    ny = 200
    xmin = -25
    xmax = 25
    ymin = -25
    ymax = 25
  []
  [new_nodeset]
    input = gen
    type = ExtraNodesetGenerator
    coord = '0 0'
    new_boundary = 100
  []
  uniform_refine = 1
[]
[GlobalParams]
  gb_width = 0.1
  int_width = 20.0
  D_b = 1.0
  D_gb0 = 1e5
  D_gb = '${fparse (15/8) * ( (D_gb0 - D_b) * gb_width / int_width ) }'
[]
[Variables] #Adds variables needed for two ways of calculating effective thermal cond.
  [T] #Temperature used for the direct calculation
    initial_condition = 800
  []
  [Tx_AEH] #Temperature used for the x-component of the AEH solve
    initial_condition = 800
    # scaling = 1.0e4 #Scales residual to improve convergence
  []
  [Ty_AEH] #Temperature used for the y-component of the AEH solve
    initial_condition = 800
    # scaling = 1.0e4 #Scales residual to improve convergence
  []
[]

[AuxVariables] #Creates second constant phase
  [h_gb]
  []

[]

[ICs] #Sets the IC for the second constant phase
  [h_gb_IC] #Creates circles with smooth interfaces at random locations
    variable = h_gb
    type = FunctionIC
    block = 0
    function = 'h_gb_func'
  []
[]

[Functions]
  [h_gb_func]
    type = ParsedFunction
    expression = '(tanh((2*x)/${GlobalParams/int_width} )^2 - 1)^2'
  []
  [D_parallel_func]
    type = ParsedFunction
    expression = '${GlobalParams/D_b} +  (${GlobalParams/D_gb} * ((tanh((2*x)/${GlobalParams/int_width} )^2 - 1)^2) )'
  []

[]

[Kernels]
  [HtCond] #Kernel for direct calculation of thermal cond
    type = AnisoHeatConduction
    variable = T
  []
  [heat_x] #All other kernels are for AEH approach to calculate thermal cond.
    type = AnisoHeatConduction
    variable = Tx_AEH
  []
  [heat_rhs_x]
    type = AnisoHomogenizedHeatConduction
    variable = Tx_AEH
    component = 0
    # diffusion_coefficient = 
  []
  [heat_y]
    type = AnisoHeatConduction
    variable = Ty_AEH
  []
  [heat_rhs_y]
    type = AnisoHomogenizedHeatConduction
    variable = Ty_AEH
    component = 1
  []
[]

[BCs]
  [Periodic]
    [all]
      auto_direction = 'x y'
      variable = 'Tx_AEH Ty_AEH'
    []
  []
  [left_T] #Fix temperature on the left side
    type = DirichletBC
    variable = T
    boundary = left
    value = 800
  []
  [right_flux] #Set heat flux on the right side
    type = NeumannBC
    variable = T
    boundary = right
    value = 5e-6
  []
  [fix_x] #Fix Tx_AEH at a single point
    type = DirichletBC
    variable = Tx_AEH
    value = 800
    boundary = 100
  []
  [fix_y] #Fix Ty_AEH at a single point
    type = DirichletBC
    variable = Ty_AEH
    value = 800
    boundary = 100
  []
[]

[Materials]
  #   [thcond] #The equation defining the thermal conductivity is defined here, using two ifs
  #     # The k in the bulk is k_b, in the precipitate k_p2, and across the interaface k_int
  #     type = ParsedMaterial
  #     block = 0
  #     constant_names = 'length_scale k_b k_p2 k_int'
  #     constant_expressions = '1e-6 5 1 0.1'
  #     expression = 'sk_b:= length_scale*k_b; sk_p2:= length_scale*k_p2; sk_int:= k_int*length_scale; if(phase2>0.1,if(phase2>0.95,sk_p2,sk_int),sk_b)'
  #     outputs = exodus
  #     f_name = thermal_conductivity
  #     coupled_variables = phase2
  #   []
  #   [phase_normal]
  #     type = PhaseNormalTensor
  #     phase = h_gb
  #     normal_tensor_name = gb_normal
  #   []
  #   [thermal_conductivity]
  #     type = GBDependentDiffusivity
  #     gb = h_gb
  #     bulk_parameter = 1.0
  #     gb_parameter = ${GlobalParams/D_gb}
  #     gb_normal_tensor_name = gb_normal
  #     gb_tensor_prop_name = thermal_conductivity
  #     outputs = exodus
  #   []

  [therm_cond]
    type = GenericFunctionRankTwoTensor
    tensor_name = 'thermal_conductivity'
    tensor_functions = '${GlobalParams/D_b} 0 0 0 D_parallel_func 0 0 0 0'
    outputs = exodus
  []

[]

[Postprocessors]
  [right_T]
    type = SideAverageValue
    variable = T
    boundary = right
  []
  #   [k_x_direct] #Effective thermal conductivity from direct method
  #     # This value is lower than the AEH value because it is impacted by second phase
  #     # on the right boundary
  #     type = ThermalConductivity
  #     variable = T
  #     flux = 5e-6
  #     length_scale = 1e-06
  #     T_hot = 800
  #     dx = 10
  #     boundary = right
  #   []
  [k_x_AEH] #Effective thermal conductivity in x-direction from AEH
    type = HomogenizedThermalConductivity
    chi = 'Tx_AEH Ty_AEH'
    row = 0
    col = 0
    is_tensor = true
    scale_factor = 1 #Scale due to length scale of problem
  []
  [k_y_AEH] #Effective thermal conductivity in x-direction from AEH
    type = HomogenizedThermalConductivity
    chi = 'Tx_AEH Ty_AEH'
    row = 1
    col = 1
    is_tensor = true
    scale_factor = 1 #Scale due to length scale of problem
  []
[]

[Preconditioning]
  [SMP]
    type = SMP
    full = true
    # off_diag_row = 'Tx_AEH Ty_AEH'
    # off_diag_column = 'Ty_AEH Tx_AEH'
  []
[]

[Executioner]
  type = Steady
  l_max_its = 30
  solve_type = NEWTON
  petsc_options_iname = '-pc_type -pc_hypre_type -ksp_gmres_restart -pc_hypre_boomeramg_strong_threshold'
  petsc_options_value = 'hypre boomeramg 31 0.7'
  l_tol = 1e-04
  nl_abs_tol = 1e-8
#   automatic_scaling = true
[]

[Outputs]
  execute_on = 'INITIAL timestep_end'
  exodus = true
  csv = true
[]
