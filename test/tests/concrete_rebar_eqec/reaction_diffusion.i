[Mesh]
  file = react_diff.e
  uniform_refine = 0
[]

[Variables]
  [temp]
  []
[]

[Executioner]
  type = Transient
  solve_type = PJFNK
  petsc_options_iname = '-pc_type -ksp_gmres_restart'
  petsc_options_value = 'lu       101'
  line_search = none
  nl_rel_tol = 1e-15
  nl_abs_tol = 1e-8
  l_max_its = 100
  nl_max_its = 10
  dt = 0.1
  end_time = 10
  # num_steps = 10
[]

[BCs]
  # [concrete_top]
  #   type = DirichletBC
  #   variable = temp
  #   boundary = 1
  #   value = 300
  # []
  [concrete_bottom]
    type = DirichletBC
    variable = temp
    boundary = 2
    value = 300
  []
  [concrete_side]
    type = DirichletBC
    variable = temp
    boundary = 3
    value = 300
  []
[]

[Outputs]
  exodus = true
  print_linear_residuals = false
[]

[Kernels]
  [HeatTdot]
    type = HeatConductionTimeDerivative
    variable = temp
  []
  [Hcond]
    type = HeatConduction
    variable = temp
    block = '1'
  []
  [Hsource]
    type = HeatSource
    variable = temp
    block = '2'
    function = 300+100*t
  []
[]

[Materials]
  [fuel_density]
    type = Density
    density = 10.0
  []
  [fthermal]
    type = HeatConductionMaterial
    temp = 'temp'
    thermal_conductivity = 5.0
    specific_heat = 1.0
  []
[]

[Constraints]
  [rebar]
    type = EqualValueEmbeddedConstraint
    slave = 2
    master = 1
    penalty = 1e3
    variable = 'temp'
    master_variable = 'temp'
    formulation = kinematic
    # model = glued
    # debug = true
  []
[]
