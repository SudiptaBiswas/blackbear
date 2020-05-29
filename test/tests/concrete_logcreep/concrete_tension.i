[MeshGenerators]
  [mesh]
    type = GeneratedMeshGenerator
    dim = 2
    nx = 40
    ny = 5
    xmin = -1.0
    xmax = 1.0
    ymin = -0.125
    ymax = 0.125
    # nz = 1
    # elem_type = HEX8
  []

  [./extra_nodeset1]
    type = ExtraNodesetGenerator
    input = mesh
    new_boundary = 'bottom_left'
    # boundary
    coord = '-1 -0.125 '
  []
  # [./extra_nodeset2]
  #   type = ExtraNodesetGenerator
  #   input = mesh
  #   new_boundary = 'top_left'
  #   coord = '0 10'
  # []
[]

[Mesh]
  type = MeshGeneratorMesh
[]

[GlobalParams]
  displacements = 'disp_x disp_y'
  volumetric_locking_correction = true
[]

[Variables]
  [./disp_x]
    order = FIRST
    family = LAGRANGE
  [../]
  [./disp_y]
    order = FIRST
    family = LAGRANGE
  [../]
  # [./disp_z]
  #   order = FIRST
  #   family = LAGRANGE
  # [../]
[]

[AuxVariables]
  [./resid_x]
  [../]
  [./resid_y]
  [../]
  [./creep_strain_xx]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./damage]
    order = CONSTANT
    family = MONOMIAL
  [../]
[]

[Modules/TensorMechanics/Master]
  [all]
    strain = finite
    # incremental = true
    # add_variables = true
    generate_output = 'stress_xx stress_xy stress_yy strain_xx strain_xy strain_yy
               max_principal_stress mid_principal_stress min_principal_stress
               secondinv_stress thirdinv_stress vonmises_stress
               secondinv_strain thirdinv_strain
               elastic_strain_xx elastic_strain_xy elastic_strain_yy'
    save_in = 'resid_x resid_y'
  []
[]

# [Kernels]
#   [./TensorMechanics]
#     displacements = 'disp_x disp_y disp_z'
#     use_displaced_mesh = true
#   [../]
# []

[AuxKernels]
  [./creep_strain_xx]
    type = RankTwoAux
    variable = creep_strain_xx
    rank_two_tensor = creep_strain
    index_j = 0
    index_i = 0
    execute_on = timestep_end
  [../]
  [damage]
    type = MaterialRealAux
    property = damage_index
    variable = damage
  []
[]

[BCs]
  [./symmy]
    type = PresetBC
    variable = disp_y
    # boundary = bottom_left
    boundary = left
    value = 0
  [../]
  [./symmx]
    type = PresetBC
    variable = disp_x
    boundary = left
    value = 0
  [../]
  # [./symmz]
  #   type = PresetBC
  #   variable = disp_z
  #   boundary = back
  #   value = 0
  # [../]

  [./right_surface]
    type = FunctionPresetBC
    variable = disp_x
    boundary = 'right'
    function = '1e-4*t'
  [../]
  # [./axial_load]
  #   type = NeumannBC
  #   variable = disp_x
  #   boundary = right
  #   value    = 10e5
  # [../]
[]

[Materials]
  [./creep]
    type = LinearViscoelasticStressUpdate
  [../]
  [./logcreep]
    type = ConcreteLogarithmicCreepModel
    poissons_ratio = 0.2
    youngs_modulus = 20e9
    recoverable_youngs_modulus = 10e9
    recoverable_viscosity = 1
    long_term_viscosity = 1
    long_term_characteristic_time = 1
  [../]
  # [./strain]
  #   type = ComputeSmallStrain
  #   displacements = 'disp_x disp_y disp_z'
  # [../]
  # [./strain]
  #   type = ComputeIncrementalSmallStrain
  #   # displacements = 'disp_x disp_y disp_z'
  # [../]
  [damage]
    type = MazarsDamage
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
    inelastic_models = 'creep'
    damage_model = damage
  [../]
[]

[UserObjects]
  [./visco_update]
    type = LinearViscoelasticityManager
    viscoelastic_model = logcreep
  [../]
[]

[Postprocessors]
  [./stress_xx]
    type = ElementAverageValue
    variable = stress_xx
    block = 'ANY_BLOCK_ID 0'
  [../]
  [./strain_xx]
    type = ElementAverageValue
    variable = strain_xx
    block = 'ANY_BLOCK_ID 0'
  [../]
  [./creep_strain_xx]
    type = ElementAverageValue
    variable = creep_strain_xx
    block = 'ANY_BLOCK_ID 0'
  [../]
  [damage]
    type = ElementAverageValue
    variable = damage
    block = 'ANY_BLOCK_ID 0'
  []
  [./deformation_x]
    type = AverageNodalVariableValue
    variable = disp_x
    boundary = 'right'
  [../]
  [./deformation_y]
    type = AverageNodalVariableValue
    variable = disp_y
    boundary = 'right'
  [../]
  [./react_x]
    type = NodalSum
    variable = resid_x
    boundary = 'left'
  [../]
  [./react_y]
    type = NodalSum
    variable = resid_y
    boundary = 'left'
  [../]
  [./react_x2]
    type = NodalSum
    variable = resid_x
    boundary = 'right'
  [../]
  [./react_y2]
    type = NodalSum
    variable = resid_y
    boundary = 'right'
  [../]
[]

[Preconditioning]
  [./smp]
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

  end_time = 25
  dtmin = 0.00001
  dt = 0.01
  # [./TimeStepper]
  #   type = LogConstantDT
  #   log_dt = 0.1
  #   first_dt = 0.1
  # [../]
[]

[Outputs]
  file_base = concrete_beam_tens_damage
  # exodus = true
  csv = true
  [./exodus]
    type = Exodus
    elemental_as_nodal = true
  [../]
[]
