
"""
EV Charging Operational Planning
Liege Intensive Course on AI for Energy Systems, May 2022
By Emmanuel AF MOMPREMIER
"""



from pyomo.environ import *


class ChargeController:

    def compute_actions(number_of_EVs, control_horizon, grid_capacity, current_time, forecast_PV, imp_price, exp_price, ev, prev_soc):
        """
        Function that computes the controller's actions regarding the charging power of the EVs, expressed as a percentage of the maximum charging power.

        :param number_of_EVS: look-ahead control horizon (hour)
        :param control_horizon: look-ahead control horizon (hour)
        :param grid_capacity: Power exchange limitation with the grid (kW)
        :param current_time: period considered, expressed as the hour of the day at which the optimal operation of the problem should start (hour)
        :param forecast_PV: Forecast of photovoltaic production over the control horizon (list[(kW)])
        :param imp_price: Import price over the control horizon (list[(€/kWh)]
        :param exp_price: Export price over the control horizon (list[(€/kWh)]
        :param ev: List of EV objects (see class EV)
        :param prev_soc: Dict of state of charge of the EVs at the previous time step (Dict[ev] = %)
        :return: EV charging setpoint (% maximum charging power) and PV generation over the control_horizon (What is this PV generation?)
        """
        
        eff = 0.9275

        imp_price = { i : imp_price[i] for i in range(0, len(imp_price) ) }
        impr_price_dict = dict()
        for i in range(control_horizon):
          impr_price_dict[i] = imp_price[i]

        forecast_PV = { i : forecast_PV[i] for i in range(0, len(forecast_PV) ) }
        forecast_PV_dict = dict()
        for i in range(control_horizon):
          forecast_PV_dict[i] = forecast_PV[i]

        exp_price = { i : exp_price[i] for i in range(0, len(exp_price) ) }
        exp_price_dict = dict()
        for i in range(control_horizon):
          exp_price_dict[i] = exp_price[i]
          
               
          #Compute efficiency as a function of charge power
          def compute_efficiency(power, max_power):
              if power >= 0 and power < 0.1*max_power:
                  eff = 0.8
              elif power >= 0.1*max_power and power < 0.8*max_power:
                  eff = 1 - (0.02*max_power)/power
              elif power >= 0.8*max_power and power <= max_power:
                  eff = 0.7 + (0.22*max_power)/power
              else:
                  return sys.exit(print('ERROR: Invalid value of efficiency.'))
              return eff


        # Initializing return values  
        Dt = 1 #time step of 1h

        #output 1
        charge_powers = dict()
       
        for e in range(number_of_EVs):
          charge_powers[e] = [0.0] * control_horizon

        #output 2
        PV_generation = [0.0] * control_horizon #real power usage from PV 
         
              
         #Creating the model, its parameters and variables
        model = ConcreteModel()
        model.IDX = Set(initialize = range(control_horizon)) #index iterable over the control_horizon i--> 7, 14
        model.cDX = Set(initialize = range(number_of_EVs))   #0,3

        model.imp_price = Param(model.IDX, initialize = impr_price_dict)
        model.exp_price = Param(model.IDX, initialize = exp_price_dict)
        model.trade_off_coefficient = Param(initialize = 500) 
        model.efficiency = Param(initialize = 0.9275)
        model.ev = Param(model.cDX, initialize = ev, within=Any)
        model.forecast_PV = Param(model.IDX, initialize = forecast_PV_dict)

        model.charge_powers = Var(model.cDX, model.IDX, domain=NonNegativeReals)
        model.PV_used = Var(model.IDX, domain = NonNegativeReals)
        model.PV_exported = Var(model.IDX, domain = NonNegativeReals)
        model.state_of_charge = Var(model.cDX, model.IDX, domain = NonNegativeReals) #bounds or not
        model.imp_quantity = Var(model.IDX, domain = NonNegativeReals)
        model.exp_quantity = Var(model.IDX, domain = NonNegativeReals)
        model.bin_export_import = Var(model.IDX, domain=Binary)
        model.pos = Var(model.cDX, domain = NonNegativeReals)
        model.neg = Var(model.cDX, domain = NonNegativeReals)
        model.penalty = Var(model.cDX, domain =  Reals )


        # Define Objective Function - Minimization
        def obj_expression(model):

          return sum((model.imp_price[i] * model.imp_quantity[i] - model.exp_price[i] * model.exp_quantity[i]) * Dt for i in model.IDX) \
          + model.trade_off_coefficient * sum( model.penalty[car] for car in model.cDX)
                          
        model.obj = Objective(rule = obj_expression, sense = minimize)
        
        
        #Attach the constraints
        def equilibrium_constraint(model, i):
          return model.imp_quantity[i] + model.PV_used[i]   == model.exp_quantity[i] + sum(model.charge_powers[car, i] for car in model.cDX)

        def PV_power_constraint(model, i):
          return model.PV_exported[i] == model.forecast_PV[i] - model.PV_used[i]

        def avoid_import_export1(model, i):
          return model.imp_quantity[i] <= grid_capacity * model.bin_export_import[i]

        def avoid_import_export2(model, i):
          return model.exp_quantity[i] <= grid_capacity * (1 - model.bin_export_import[i])

        def penalty_constraint(model, car):
          return model.penalty[car] ==  (model.ev[car].departure_time - model.ev[car].arrival_time)/ model.ev[car].capacity  * model.ev[car].price * (model.pos[car] + model.neg[car])
            
        def state_of_charge_constraint(model, i, car):

          if i == 0:
              if model.ev[car].state != 'connected':
                  return model.state_of_charge[car, i] == model.ev[car].arrival_soc
              else:
                  return model.state_of_charge[car, i] == model.ev[car].arrival_soc + model.charge_powers[car, i] * model.efficiency * Dt / model.ev[car].capacity
                     
          else:
              if i < model.ev[car].arrival_time - current_time:
                  return model.state_of_charge[car, i] == model.state_of_charge[car, i-1]
              elif i == model.ev[car].arrival_time - current_time:
                  return model.state_of_charge[car, i] == model.ev[car].arrival_soc + model.charge_powers[car, i] * model.efficiency * Dt / model.ev[car].capacity
              elif i > model.ev[car].arrival_time - current_time and i <= model.ev[car].departure_time - current_time:
                  return model.state_of_charge[car, i] == model.state_of_charge[car, i-1] + model.charge_powers[car, i] * model.efficiency * Dt / model.ev[car].capacity
              else:
                  return model.state_of_charge[car, i] == model.state_of_charge[car, i-1]

        def absolute_value_constraint(model, car):
          if model.ev[car].departure_time - current_time >= 0:
              return model.pos[car] - model.neg[car]  == model.state_of_charge[car, model.ev[car].departure_time - current_time ] - model.ev[car].departure_soc \
                      
          else:
              return model.pos[car] + model.neg[car] == 0

        model.constraint1 = Constraint(model.IDX, rule=equilibrium_constraint)
        model.constraint2 = Constraint(model.IDX, rule = PV_power_constraint)
        model.constraint3 = Constraint(model.IDX, rule = avoid_import_export1)
        model.constraint4 = Constraint(model.IDX, rule = avoid_import_export2)
        model.constraint5 = Constraint(model.IDX, model.cDX, rule = state_of_charge_constraint)
        model.constraint6 = Constraint(model.cDX, rule = absolute_value_constraint)

        model.constraint7 = ConstraintList()   
        for car in model.cDX:
            for i in model.IDX:
                model.constraint7.add(expr = model.charge_powers[car,i] <= model.ev[car].max_charging_power) #model.ev[car].max_charging_power

        model.constraint8 = Constraint(model.cDX, rule = penalty_constraint)



        # Code snipet for calling the solver 
        solver = SolverFactory("cbc")
        results = solver.solve(model, tee=False) # tee=True makes the solver verbose

        if (results.solver.status == SolverStatus.ok) and (results.solver.termination_condition == TerminationCondition.optimal):
            pass # Do something when the solution is optimal and feasible
        elif (results.solver.termination_condition == TerminationCondition.infeasible):
            print (">>> INFEASIBLE MODEL dumped to tmp.lp")
            model.write("tmp.lp", io_options={'symbolic_solver_labels': True}) # Export the model
        else:
            # Something else is wrong
            print("Solver Status: ",  results.solver.status)
            print (">>> MODEL dumped to strange.lp")
            model.write("strange.lp", io_options={'symbolic_solver_labels': True}) # Export the model

        for i in model.IDX:
            for car in model.cDX:
                charge_powers[car][i] = (value(model.charge_powers[car, i]) / value(model.ev[car].max_charging_power) )
                PV_generation[i] = value(model.PV_used[i])
            
            
        return charge_powers, PV_generation
