from gurobipy import GRB, Model

#import impedance
#import settings
#import road
import gurobipy as gp
"""
model = gp.Model("bakery_optimization")

cakes = model.addVar(vtype = GRB.INTEGER, lb = 0, ub = 100, name = "cakes")
bread = model.addVar(vtype = GRB.INTEGER, lb = 0, ub = 100, name = "bread")

model.setObjective(5 * cakes + 2 * bread, GRB.MAXIMIZE)

model.addConstr(0.5 * bread + 2 * cakes <= 100, name = "flour_limit") # 100kg flour limit
model.addConstr(cakes + bread <= 80, name = "oven_capacity") # oven capacity is 80

model.optimize()

if model.status == GRB.OPTIMAL:
    print(f"optimal profit: ${model.objVal:2f}")
    print(f"Bakes {bread.X:.2f} loaves of bread")
    print(f"Bakes {cakes.X:.2f} cakes")


else:
    print("No optimal solution found")
"""

"""
model = gp.Model("combined_sum_of_units")

items_x = model.addVar(vtype = GRB.INTEGER, lb = 45.0, name = "items produced with X")
items_y = model.addVar(vtype = GRB.INTEGER, lb = 5.0, name = "items produced with Y")

model.setObjective(items_x + items_y - 50, GRB.MAXIMIZE)

model.addConstr(50 * items_x + 24 * items_y <= 40 * 60, name = "limit on machine X")
model.addConstr(30 * items_x + 33 * items_y <= 35 * 60, name = "limit on machine Y")

model.optimize()

if model.status == GRB.OPTIMAL:
    print(f"number of x items: {items_x.X:.2f}")
    print(f"number of y items: {items_y.X:.2f}") 
else:
    print("No optimal solution found")
"""