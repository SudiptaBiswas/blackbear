[Mesh]
  file = RCBeam_test.e
  # Units: N,m
  # block 1 beam
  # block 2 #8 longitudinal reinforcement
  # block 3 #5 longitudinal reinforcement
  # block 4 #3 transverse reinforcement
  # block 5 #5 transverse reinforcement
  # sideset 1 loading boundary
  # sideset 2 right suppport
  # sideset 3 left support
  # sideset 4 clamping force
[]

[GlobalParams]
  displacements = 'disp_x disp_y'
  volumetric_locking_correction = true
[]

[Modules/TensorMechanics/Master]
  [./Concrete_block]
    block = 1
    # strain = small
    strain = finite
    # incremental = true
   # add_variables = true
    generate_output = 'stress_xx stress_xy stress_yy strain_xx strain_xy strain_yy
    		       max_principal_stress mid_principal_stress min_principal_stress
    		       secondinv_stress thirdinv_stress vonmises_stress
    		       secondinv_strain thirdinv_strain
    		       elastic_strain_xx elastic_strain_xy elastic_strain_yy'
#    		       plastic_strain_xx plastic_strain_xy plas tic_strain_xz plastic_strain_yy plastic_strain_yz plastic_strain_zz'
    save_in = 'resid_x resid_y'
  [../]
[]

[Modules/TensorMechanics/LineElementMaster]
  [./Reinforcement_block]
    block = '2 3'
    truss = true
    area = area
    displacements = 'disp_x disp_y'
    save_in = 'resid_x resid_y'
   # add_variables = true
  [../]
[]

# [Kernels]
#   [./solid_x]
#     type = StressDivergenceTensorsTruss
#     block = '2 3'
#     displacements = 'disp_x'
#     component = 0
#     variable = disp_x
#     area = area
#     # save_in = react_x
#   [../]
#   # [./solid_y]
#   #   type = StressDivergenceTensorsTruss
#   #   block = '1 2'
#   #   displacements = 'disp_x disp_y disp_z'
#   #   component = 1
#   #   variable = disp_y
#   #   area = area
#   #   save_in = react_y
#   # [../]
#   # [./solid_z]
#   #   type = StressDivergenceTensorsTruss
#   #   block = '1 2'
#   #   displacements = 'disp_x disp_y disp_z'
#   #   component = 2
#   #   variable = disp_z
#   #   area = area
#   #   save_in = react_z
#   # [../]
# []

[Variables]
  [./disp_x]
  [../]
  [./disp_y]
  [../]
[]

[AuxVariables]
  [./resid_x]
  [../]
  [./resid_y]
  [../]
  [./area]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./axial_stress]
    order = CONSTANT
    family = MONOMIAL
  [../]
[]

[AuxKernels]
  [./area]
    type = ConstantAux
    block = '2 3'
    variable = area
    value = 2.00e-4 # 509 mm2
    execute_on = 'initial timestep_begin'
  [../]
  [./axial_stress]
    type = MaterialRealAux
    block = '2 3'
    variable = axial_stress
    property = axial_stress
  [../]
  # [./area3]
  #   type = ConstantAux
  #   block = 3
  #   variable = area
  #   value = 2.00e-4 # 200 mm2
  #   execute_on = 'initial timestep_begin'
  # [../]
[]

[Constraints]
  [./rebar_x2]
    type = EqualValueEmbeddedConstraint
    slave = 2
    master = 1
    penalty = 1e11
    variable = 'disp_x'
    master_variable = 'disp_x'
    formulation = penalty
  [../]
  [./rebar_y2]
    type = EqualValueEmbeddedConstraint
    slave = 2
    master = 1
    penalty = 1e11
    variable = 'disp_y'
    master_variable = 'disp_y'
    formulation = penalty
  [../]
  [./rebar_x3]
    type = EqualValueEmbeddedConstraint
    slave = 3
    master = 1
    penalty = 1e11
    variable = 'disp_x'
    master_variable = 'disp_x'
    formulation = penalty
  [../]
  [./rebar_y3]
    type = EqualValueEmbeddedConstraint
    slave = 3
    master = 1
    penalty = 1e11
    variable = 'disp_y'
    master_variable = 'disp_y'
    formulation = penalty
  [../]
[]

[BCs]
  [./loading]
    type = FunctionPresetBC
    variable = disp_x
    boundary = '2'
    function = '-1e-5*t'
  [../]
  [./left_support_x]
    type = PresetBC
    variable = disp_x
    boundary = '1'
    value = 0
  [../]
  [./left_support_y]
    type = PresetBC
    variable = disp_y
    boundary = '100'
    value = 0
  [../]
[]

[Postprocessors]
  [./deformation_x]
    type = AverageNodalVariableValue
    variable = disp_x
    boundary = '2'
  [../]
  [./react_x]
    type = NodalSum
    variable = resid_x
    boundary = '1'
  [../]
  [./stress_xx]
    type = ElementAverageValue
    variable = stress_xx
    block = '1'
  [../]
  [./strain_xx]
    type = ElementAverageValue
    variable = strain_xx
    block = '1'
  [../]
  # [./creep_strain_xx]
  #   type = ElementAverageValue
  #   variable = creep_strain_xx
  #   block = '1'
  # [../]
  [./axial_stress]
    type = ElementAverageValue
    variable = axial_stress
    block = '2'
  [../]
[]

[Materials]
  # Material Properties of concrete
  [./creep]
    type = LinearViscoelasticStressUpdate
    block = 1
  [../]
  [./logcreep]
    type = ConcreteLogarithmicCreepModel
    block = 1
    poissons_ratio = 0.2
    youngs_modulus = 20e9
    recoverable_youngs_modulus = 10e9
    recoverable_viscosity = 1
    long_term_viscosity = 1
    long_term_characteristic_time = 1
  [../]
  [damage]
    type = MazarsDamage
    block = 1
    tensile_strength = 1e6
    a_t = 0.87
    a_c = 0.65
    b_t = 20000
    b_c = 2150
    use_old_damage = true
    outputs = exodus
  []
  [./stress]
    type = ComputeMultipleInelasticStress
    block = 1
    inelastic_models = 'creep'
    damage_model = damage
  [../]
  # Material Properties of steel reinforcement
  # [./truss]
  #   type = LinearElasticTruss
  #   block = '2 3'
  #   youngs_modulus = 2e11
  # [../]
  [./truss]
    type = PlasticityTruss
    block = '2 3'
    youngs_modulus = 200e9
    poissons_ratio = 0.3
    hardening_constant = 1.0
    yield_strength = 5e6
    # tolerance = 1e-8
    displacements = 'disp_x'
    outputs = exodus
  [../]
[]

[UserObjects]
  [./visco_update]
    type = LinearViscoelasticityManager
    block = 1
    viscoelastic_model = logcreep
  [../]
[]

# [Postprocessors]
#   [./stress_xx]
#     type = ElementAverageValue
#     variable = stress_xx
#     block = 'ANY_BLOCK_ID'
#   [../]
#   [./strain_xx]
#     type = ElementAverageValue
#     variable = strain_xx
#     block = 'ANY_BLOCK_ID'
#   [../]
#   [./creep_strain_xx]
#     type = ElementAverageValue
#     variable = creep_strain_xx
#     block = 'ANY_BLOCK_ID'
#   [../]
#   [damage]
#     type = ElementAverageValue
#     variable = damage
#     block = 'ANY_BLOCK_ID'
#   []
# []

[Preconditioning]
  [./SMP]
    type = SMP
    full = true
  [../]
[]

[Executioner]
  type = Transient
  solve_type = 'NEWTON'
  nl_max_its = 100
  nl_abs_tol = 1.E-5
  nl_rel_tol = 1E-3

  line_search = none

  petsc_options_iname = '-pc_type'
  petsc_options_value = 'lu'

  petsc_options = '-snes_converged_reason'

  end_time = 200
  dtmin = 0.01

  # dt = 0.1
  [./TimeStepper]
    type = LogConstantDT
    log_dt = 0.1
    first_dt = 0.1
  [../]
  # type = Transient
  #
  # l_max_its  = 50
  # l_tol      = 1e-8
  # nl_max_its = 20
  # nl_rel_tol = 1e-10
  # nl_abs_tol = 1e-8
  #
  # dtmin = 0.01
  # end_time = 100
  # [./TimeStepper]
  #   type = LogConstantDT
  #   log_dt = 0.1
  #   first_dt = 0.1
  # [../]
[]


[Outputs]
  # print_linear_residuals = false
  exodus = true
  csv = true
[]
