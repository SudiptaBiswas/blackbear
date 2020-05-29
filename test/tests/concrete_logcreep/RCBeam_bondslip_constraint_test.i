[Mesh]
  [./cont]
    type = GeneratedMeshGenerator
    dim = 2
    nx = 1
    ny = 1
    ymax = 0.1
  [../]
  [./cont_sidesets]
    type = RenameBoundaryGenerator
    input = cont
    old_boundary_id = '0 1 2 3'
    new_boundary_name = 'cont_bottom cont_right cont_top cont_left'
  [../]
  [./cont_id]
    type = SubdomainIDGenerator
    input = cont_sidesets
    subdomain_id = 1
  [../]
  [./truss]
    type = GeneratedMeshGenerator
    dim = 1
    elem_type = EDGE
    nx = 2
    # xmin = -1
 [../]
 # [./truss_sidesets]
 #   type = RenameBoundaryGenerator
 #   input = truss
 #   old_boundary_id = '0 1'
 #   new_boundary_name = 'truss_left truss_right'
 # [../]
 [./truss_id]
   type = SubdomainIDGenerator
   input = truss
   subdomain_id = 2
 [../]

 [./combined]
   type = MeshCollectionGenerator
   inputs = 'cont_id truss_id'
 [../]
[]

[GlobalParams]
  displacements = 'disp_x disp_y'
  # volumetric_locking_correction = true
[]

[Modules/TensorMechanics/Master]
  [./Concrete_block]
    block = 1
    strain = small
    # strain = finite
    incremental = true
   # add_variables = true
    # generate_output = 'stress_xx stress_xy stress_yy strain_xx strain_xy strain_yy
    # 		       max_principal_stress mid_principal_stress min_principal_stress
    # 		       secondinv_stress thirdinv_stress vonmises_stress
    # 		       secondinv_strain thirdinv_strain
    # 		       elastic_strain_xx elastic_strain_xy elastic_strain_yy'
#    		       plastic_strain_xx plastic_strain_xy plas tic_strain_xz plastic_strain_yy plastic_strain_yz plastic_strain_zz'
    save_in = 'resid_x resid_y'
  [../]
[]

[Kernels]
  [./solid]
    type = StressDivergenceTensorsTruss
    component = 0
    variable = disp_x
    area = area
    # save_in = react_x
    block = 2
  [../]
  [./solidy]
    type = StressDivergenceTensorsTruss
    component = 1
    variable = disp_y
    area = area
    # save_in = react_x
    block = 2
  [../]
[]

# [Modules/TensorMechanics/LineElementMaster]
#   [./Reinforcement_block]
#     block = '2'
#     truss = true
#     area = area
#     displacements = 'disp_x disp_y'
#     save_in = 'resid_x resid_y'
#    # add_variables = true
#   [../]
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
  # [./damage]
  #   order = CONSTANT
  #   family = MONOMIAL
  # [../]
[]

[AuxKernels]
  [./area]
    type = ConstantAux
    block = '2'
    variable = area
    value = 2.00e-4 # 509 mm2
    execute_on = 'initial timestep_begin'
  [../]
  [./axial_stress]
    type = MaterialRealAux
    block = '2'
    variable = axial_stress
    property = axial_stress
  [../]
  # [damage]
  #   type = MaterialRealAux
  #   block = '1'
  #   property = damage_index
  #   variable = damage
  # []
  # [./area3]
  #   type = ConstantAux
  #   block = 3
  #   variable = area
  #   value = 2.00e-4 # 200 mm2
  #   execute_on = 'initial timestep_begin'
  # [../]
[]

# [Constraints]
#   [./rebar_x2]
#     type = EqualValueEmbeddedConstraint
#     slave = 2
#     master = 1
#     penalty = 1e11
#     variable = 'disp_x'
#     master_variable = 'disp_x'
#     formulation = penalty
#   [../]
#   [./rebar_y2]
#     type = EqualValueEmbeddedConstraint
#     slave = 2
#     master = 1
#     penalty = 1e11
#     variable = 'disp_y'
#     master_variable = 'disp_y'
#     formulation = penalty
#   [../]
#   [./rebar_x3]
#     type = EqualValueEmbeddedConstraint
#     slave = 3
#     master = 1
#     penalty = 1e11
#     variable = 'disp_x'
#     master_variable = 'disp_x'
#     formulation = penalty
#   [../]
#   [./rebar_y3]
#     type = EqualValueEmbeddedConstraint
#     slave = 3
#     master = 1
#     penalty = 1e11
#     variable = 'disp_y'
#     master_variable = 'disp_y'
#     formulation = penalty
#   [../]
# []

[Constraints]
  [rebar_x]
    type = RebarBondSlipConstraint
    slave = 2
    master = 1
    penalty = 1e6
    variable = 'disp_x'
    master_variable = 'disp_x'
    component = 0
    max_bondstress = 1e6
    transitional_slip_values = 0.05e-3
    ultimate_slip = 0.1
    rebar_radius = 7.98e-3
    debug = true
  []
  [rebar_y]
    type = RebarBondSlipConstraint
    slave = 2
    master = 1
    penalty = 1e6
    variable = 'disp_y'
    master_variable = 'disp_y'
    component = 1
    max_bondstress = 1e6
    transitional_slip_values = 0.05e-3
    ultimate_slip = 0.1
    rebar_radius = 7.98e-3
  []
  # [./rebar_y3]
  #     type = EqualValueEmbeddedConstraint
  #     slave = 3
  #     master = 1
  #     penalty = 1e6
  #     variable = 'disp_y'
  #     master_variable = 'disp_y'
  #     formulation = penalty
  #   [../]
  # [rebar_x2]
  #   type = RebarBondSlipConstraint
  #   slave = 3
  #   master = 1
  #   penalty = 1e8
  #   variable = 'disp_x'
  #   master_variable = 'disp_x'
  #   component = 0
  #   max_bondstress = 100.0
  #   transitional_slip_values = 0.0005
  #   ultimate_slip = 0.1
  #   # rebar_radius = 1.0
  # []
  # [rebar_y2]
  #   type = RebarBondSlipConstraint
  #   slave = 3
  #   master = 1
  #   penalty = 1e8
  #   variable = 'disp_y'
  #   master_variable = 'disp_y'
  #   component = 1
  #   max_bondstress = 100.0
  #   transitional_slip_values = 0.0005
  #   ultimate_slip = 0.1
  # []
[]

[Functions]
  [./loading]
    type = PiecewiseLinear
    # x = '0 1       2     3 '
    # y = '0 0.001 -0.0001 0.0'
    x = '0 1'
    y = '0 -0.0002'
  [../]
[]

[BCs]
  # [./loading]
  #   type = FunctionDirichletBC
  #   # type = DirichletBC
  #   variable = disp_x
  #   boundary = '102'
  #   function = loading
  #   # value = 0.00004
  #   preset = true
  # [../]
  # [./left_support_x]
  #   type = DirichletBC
  #   variable = disp_x
  #   boundary = '100'
  #   value = 0
  #   # preset = true
  # [../]
  # [./left_support_y]
  #   type = DirichletBC
  #   variable = disp_y
  #   # boundary = '100'
  #   # boundary = '4'
  #   boundary = '100'
  #   value = 0
  # [../]
  [./fixx]
    type = DirichletBC
    variable = disp_x
    boundary = 'cont_left'
    value = 0.0
  [../]
  [./fixy]
    type = DirichletBC
    variable = disp_y
    boundary = cont_left
    value = 0.0
  [../]
  [./load]
    type = FunctionDirichletBC
    variable = disp_x
    boundary = cont_right
    function = 'loading'
    preset = true
  [../]
  [./fixx_truss]
    type = DirichletBC
    variable = disp_x
    boundary = 0
    value = 0.0
  [../]
  [./fixy_truss]
    type = DirichletBC
    variable = disp_y
    boundary = 0
    value = 0.0
  [../]
[]

# [Postprocessors]
#   [./deformation_x]
#     type = AverageNodalVariableValue
#     variable = disp_x
#     boundary = '101'
#   [../]
#   [./deformation_y]
#     type = AverageNodalVariableValue
#     variable = disp_y
#     boundary = '101'
#   [../]
#   [./react_x]
#     type = AverageNodalVariableValue
#     variable = resid_x
#     boundary = '100'
#   [../]
#   [./react_y]
#     type = AverageNodalVariableValue
#     variable = resid_y
#     boundary = '100'
#   [../]
#   [./react_x2]
#     type = AverageNodalVariableValue
#     variable = resid_x
#     boundary = '101'
#   [../]
#   [./react_y2]
#     type = AverageNodalVariableValue
#     variable = resid_y
#     boundary = '100'
#   [../]
#   # [./node1_fx]
#   #   type = AverageNodalVariableValue
#   #   variable = resid_x
#   #   boundary = '1001'
#   #   # nodeid = 323
#   # [../]
#   [./node1_fx]
#     type = NodalVariableValue
#     variable = resid_x
#     # boundary = '1001'
#     nodeid = 138
#   [../]
#   [./node1_fy]
#     type = NodalVariableValue
#     variable = resid_y
#     # boundary = '1001'
#     nodeid = 138
#   [../]
#   [./node1_dx]
#     type = NodalVariableValue
#     variable = disp_x
#     # boundary = '1001'
#     nodeid = 138
#   [../]
#   [./node1_dy]
#     type = NodalVariableValue
#     variable = disp_y
#     # boundary = '1001'
#     nodeid = 138
#   [../]
#   [./node1_fx2]
#     type = AverageNodalVariableValue
#     variable = resid_x
#     boundary = '102'
#     # nodeid = 323
#   [../]
#   # [./node2_fx]
#   #   type = AverageNodalVariableValue
#   #   variable = resid_x
#   #   boundary = '101'
#   #   # nodeid = 99
#   # [../]
#   # [./node2_fy]
#   #   type = NodalVariableValue
#   #   variable = resid_y
#   #   nodeid = 99
#   # [../]
#   # [./node1_ux]
#   #   type = AverageNodalVariableValue
#   #   variable = disp_x
#   #   boundary = '1001'
#   #   # nodeid = 323
#   # [../]
#   # [./node1_uy]
#   #   type = AverageNodalVariableValue
#   #   variable = disp_y
#   #   boundary = '1001'
#   #   # nodeid = 323
#   # [../]
#   # [./node2_ux]
#   #   type = AverageNodalVariableValue
#   #   variable = disp_x
#   #   boundary = '1002'
#   #   # nodeid = 99
#   # [../]
#   # [./node2_ux2]
#   #   type = NodalVariableValue
#   #   variable = disp_x
#   #   nodeid = 99
#   # [../]
#
#   [./stress_xx]
#     type = ElementAverageValue
#     variable = stress_xx
#     block = '1'
#   [../]
#   [./strain_xx]
#     type = ElementAverageValue
#     variable = strain_xx
#     block = '1'
#   [../]
#   # [./plastic_strain_truss]
#   #   type = ElementAverageValue
#   #   variable = plastic_strain
#   #   block = '2'
#   # [../]
#   [./axial_stress]
#     type = ElementAverageValue
#     variable = axial_stress
#     block = '2'
#   [../]
#   # [damage]
#   #   type = ElementAverageValue
#   #   variable = damage
#   #   block = '1'
#   # []
# []

[Materials]
  # Material Properties of concrete
  # [./creep]
  #   type = LinearViscoelasticStressUpdate
  #   block = 1
  # [../]
  # [./logcreep]
  #   type = ConcreteLogarithmicCreepModel
  #   block = 1
  #   poissons_ratio = 0.2
  #   youngs_modulus = 20e9
  #   recoverable_youngs_modulus = 10e9
  #   recoverable_viscosity = 1
  #   long_term_viscosity = 1
  #   long_term_characteristic_time = 1
  # [../]
  # [damage]
  #   type = MazarsDamage
  #   block = 1
  #   tensile_strength = 1e6
  #   a_t = 0.87
  #   a_c = 0.65
  #   b_t = 20000
  #   b_c = 2150
  #   use_old_damage = true
  #   outputs = exodus
  # []
  # [damage]
  #   type = MazarsDamage
  #   use_old_damage = true
  #   tensile_strength = 1e6
  #   a_t = 0.87
  #   a_c = 0.65
  #   b_t = 20000
  #   b_c = 2150
  #   block = 1
  # []
  # [concrete_stress]
  #   type = ComputeFiniteStrainElasticStress
  #   # damage_model = damage
  #   block = 1
  # []
  # [./stress]
  #   type = ComputeMultipleInelasticStress
  #   block = 1
  #   inelastic_models = 'creep'
  #   damage_model = damage
  # [../]
  # Material Properties of steel reinforcement
  # [./truss]
  #   type = LinearElasticTruss
  #   block = '2 3'
  #   youngs_modulus = 2e11
  # [../]
  # [./linelast]
  #   type = PlasticTruss
  #   block = '2'
  #   youngs_modulus = 2.0e11
  #   poissons_ratio = 0.3
  #   hardening_constant = 0.0
  #   yield_stress = 500e5
  #   # tolerance = 1e-8
  #   displacements = 'disp_x disp_y'
  #   outputs = exodus
  # [../]
  [Cijkl_concrete]
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = 500e6
    poissons_ratio = 0.2
    block = 1
  []
  [./isotropic_plasticity]
    type = IsotropicPlasticityStressUpdate
    yield_stress = 285788383.2488647 # = sqrt(3)*165e6 = sqrt(3) * yield in shear
    hardening_constant = 0.0
    block = '1'
  [../]
  [./radial_return_stress]
    type = ComputeMultipleInelasticStress
    tangent_operator = elastic
    inelastic_models = 'isotropic_plasticity'
    block = '1'
  [../]
  [truss]
    type = LinearElasticTruss
    block = '2'
    youngs_modulus = 2e11
  []
[]

# [UserObjects]
#   [./visco_update]
#     type = LinearViscoelasticityManager
#     block = 1
#     viscoelastic_model = logcreep
#   [../]
# []

[Preconditioning]
  [./SMP]
    type = SMP
    full = true
  [../]
[]

[Executioner]
  type = Transient
  solve_type = 'PJFNK'
  nl_max_its = 100
  nl_abs_tol = 1e-05
  nl_rel_tol = 1e-05
  l_tol = 1e-03

  line_search = none

  petsc_options_iname = '-pc_type'
  petsc_options_value = 'lu'

  petsc_options = '-snes_converged_reason'

  end_time = 1
  dtmin = 0.00001

  dt = 0.1
  # num_steps = 3
  # [./TimeStepper]
  #   type = LogConstantDT
  #   log_dt = 0.01
  #   first_dt = 0.01
  # [../]
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
  # file_base = RCBeam_bondslip_test
[]
