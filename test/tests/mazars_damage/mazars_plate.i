# Test MazarsDamage model in uniaxial tension and compression.
# This test is a single element with imposed Dirichlet BCs to
# pull or push on the element and verify that it follows the
# expected hardening/softening behavior.

[GlobalParams]
  displacements = 'disp_x disp_y'
[]

[Mesh]
  type = GeneratedMesh
  dim = 2
  nx = 10
  ny = 10
  xmax = 2
  ymax = 2
  # nz = 1
  # elem_type = HEX8
[]

[AuxVariables]
  [stress_xx]
    order = CONSTANT
    family = MONOMIAL
  []
  [strain_xx]
    order = CONSTANT
    family = MONOMIAL
  []
  [damage_index]
    order = CONSTANT
    family = MONOMIAL
  []
[]

[Modules/TensorMechanics/Master]
  [all]
    strain = SMALL
    incremental = true
    add_variables = true
  []
[]

[AuxKernels]
  [stress_xx]
    type = RankTwoAux
    variable = stress_xx
    rank_two_tensor = stress
    index_j = 0
    index_i = 0
    execute_on = timestep_end
  []
  [strain_xx]
    type = RankTwoAux
    variable = strain_xx
    rank_two_tensor = total_strain
    index_j = 0
    index_i = 0
    execute_on = timestep_end
  []
  [damage_index]
    type = MaterialRealAux
    variable = damage_index
    property = damage_index
    execute_on = timestep_end
  []
[]

[BCs]
  [symmy]
    type = PresetBC
    variable = disp_y
    boundary = bottom
    value = 0
  []
  [symmx]
    type = PresetBC
    variable = disp_x
    boundary = left
    value = 0
  []
  # [symmz]
  #   type = PresetBC
  #   variable = disp_z
  #   boundary = back
  #   value = 0
  # []
  [axial_load]
    type = FunctionPresetBC
    variable = disp_x
    boundary = right
    function = pull
  []
[]

[Functions]
  [push]
    type = ParsedFunction
    value = '-0.01*t'
  []
  [pull]
    type = ParsedFunction
    value = '0.1*t'
  []
[]

[Materials]
  [damage]
    type = MazarsDamage
    tensile_strength = 1e6
    a_t = 0.87
    a_c = 0.65
    b_t = 20000
    b_c = 2150
    use_old_dmage = true
  []

    # [stress]
    #   type = ComputeDamageStress
    #   damage_model = damage
    # []
  [./stress]
    type = ComputeMultipleInelasticStress
    inelastic_models = 'creep'
    damage_model = damage
    # use_old_damage = true
  [../]
  # [elasticity]
  #   type = ComputeIsotropicElasticityTensor
  #   poissons_ratio = 0.2
  #   youngs_modulus = 10e9
  # []
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
[]

[UserObjects]
  [./visco_update]
    type = LinearViscoelasticityManager
    viscoelastic_model = logcreep
  [../]
[]

[Postprocessors]
  [stress_xx]
    type = ElementAverageValue
    variable = stress_xx
  []
  [strain_xx]
    type = ElementAverageValue
    variable = strain_xx
  []
  [damage_index]
    type = ElementAverageValue
    variable = damage_index
  []
[]

[Preconditioning]
  [smp]
    type = SMP
    full = true
  []
[]

[Executioner]
  type = Transient

  l_max_its  = 50
  l_tol      = 1e-8
  nl_max_its = 20
  nl_rel_tol = 1e-10
  nl_abs_tol = 1e-8

  # dt = 0.1
  dtmin = 0.01
  end_time = 10
  [./TimeStepper]
    type = LogConstantDT
    log_dt = 0.1
    first_dt = 0.1
  [../]
[]

[Outputs]
  exodus = true
  csv = true
  file_base = creep_damage_tension_plate
[]
