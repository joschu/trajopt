import numpy as np
from time import time

############################
###### alg parameters ######
############################

# todo: figure out how to pass these around sensibly

barrier_start = .1 # t parameter
n_barrier_iter = 5
barrier_ratio = 10
n_newton_iter = 30
max_backtrack_iter = 10
backtrack_start = 1
backtrack_alpha = .5 
backtrack_beta = .1

def get_start_vec(lb, ub):
    out = np.empty(lb.size)
    for i in xrange(lb.size):
        if lb[i] > -1e6:
            if ub[i] < 1e6:
                out[i] = (lb[i] + ub[i])/2
            else:
                out[i] = lb[i] + 1
        else:
            if ub[i] < 1e6:
                out[i] = ub[i] - 1
            else:
                out[i] = 0
    return out
                
            

def solve_qp(H, f, A, b, C, d, lb, ub, xstart=None):
    if not xstart: xstart = get_start_vec(lb, ub)
    else: xstart = xstart.copy()
    
    nvars = H.shape[0]
    assert H.shape == (nvars, nvars)
    assert f.shape == (nvars,)
    assert A.shape[1] == nvars
    assert b.shape == (A.shape[0],)
    assert C.shape[1] == nvars
    assert d.shape == (C.shape[0],)
    assert lb.shape == (nvars,)
    assert ub.shape == (nvars,)
    assert xstart.shape == (nvars,)
    
    
    
   #H1 = np.triu(H) + np.triu(H.T, k=1)
    H1 = H.copy()
    f1 = f.copy()
    C1 = C.copy()
    d1 = d.copy()
    
    elim_inds = []
    
    for (irow,row) in enumerate(A):
        i_elim = np.flatnonzero(row)[-1]
        elim_inds.append(i_elim)
        zero_expr = row/row[i_elim]
        zero_const = d[irow]
        
        without_elim = zero_expr.copy()
        without_elim[i_elim] = 0
        H1[i_elim, i_elim] = 0
        H1 += np.outer(without_elim, without_elim)
        
        # xxx need to also eliminate linear terms in general
        # xxx also get constant term
        
        f1 -= zero_expr * f[i_elim]
        
        for (icrow,crow) in enumerate(C1): 
            if crow[i_elim] != 0:
                C1[icrow] -= zero_expr * crow[i_elim]
                d[icrow] -= zero_const
    #assert (H1[elim_inds][:,elim_inds]==0).all()
    #assert (C1[:,elim_inds]==0).all()
    #assert (f1[elim_inds] == 0).all()
    
    for (icrow, crow) in enumerate(C1):
        i_var = np.flatnonzero(crow)[-1]
        xstart[i_var] = 1 + (crow[:i_var].dot(xstart[:i_var]) - d[icrow])/crow[i_var]
    
    keep_inds = np.setdiff1d(range(H.shape[0]), elim_inds)
    xreduced = solve_iqp(H1[keep_inds][:,keep_inds], f1[keep_inds], C1[:,keep_inds], d1, lb[keep_inds], ub[keep_inds], xstart[keep_inds])
    
                
        

def solve_iqp(H, f, C, d, lb, ub, xstart):
    """
    minimize f(x) (1/2) * x^T H x + f^T x
    subject to
    Cx <= d
    
    Outer barrier loop: solve a sequence of barrier problems
        min f_t(x) = t f(x) + phi(x)
        where phi(x) = sum_i log( C_i . x - d_i )
    Newton's method for each barrier problem:
        do a bunch of newton steps until convergence
    Newton steps
        Linearize the problem to Q dx + g dx = 0
        Solve it
        Do a line search using the calculated step    
    """

    nvars = H.shape[0]
    assert H.shape == (nvars, nvars)
    assert f.shape == (nvars,)
    assert C.shape[1] == nvars
    assert lb.shape == (nvars,)
    assert ub.shape == (nvars,)
    assert xstart.shape == (nvars,)
    assert d.shape == (C.shape[0],)
    
    
    #    XXXXXX BAD
    ub = np.fmin(ub, 100)
    lb = np.fmin(lb, 100)
    # so it's nonsingular
    
    #####################
    #### moving params ##
    #####################
    t = barrier_start
    x = xstart
    
    
    ##### XXX pass constraints all the way to line search!
    
    for barrier_iter in xrange(n_barrier_iter):
        t *= barrier_ratio
        print barrier_iter
        print x
        print t
        
        def barrier_objective(x):
            return t * (.5*x.dot(H).dot(x) + f.dot(x)) + np.log(x - lb).sum() + np.log(ub - x).sum() + np.log(d - C.dot(x)).sum()
        def constraint_interior(x):
            return (x > lb).all() and (x < ub).all() and (C.dot(x) < d).all()
        def barrier_grad(x):
            return t * (H.dot(x) + f) + (1/(ub - x)) + (1/(x - lb)).sum() + (1/(d - C.dot(x))).dot(C)
        def barrier_hessian(x):
            out = t*H
            for (icrow, crow) in enumerate(C):
                out += np.outer(crow, crow) / (crow.dot(x) - d[icrow])**2
            out += np.diag((x-lb)**-2) + np.diag((ub-x)**-2)
            return out
        assert constraint_interior(x)
        x = newton_method_solve(barrier_objective, barrier_grad, barrier_hessian, constraint_interior, x)
    return x
        
def newton_method_solve(f, grad_func, hess_func, interior_func, xstart):
    x = xstart
    for i in xrange(n_newton_iter):
        grad = grad_func(x)
        hess = hess_func(x)
        xstep = linear_solve(hess, -grad)
        x = line_search(f, grad, interior_func, xstep, x)
    return x
      
def line_search(f, gradx, interior_func, xstep, x):
    """
    min_s f(x + s*xstep)
    return optimal x
    """
    s = backtrack_start
    backtrack_iter = 0
    yold = f(x)
    while True:
        xnew = x + s * xstep
        if interior_func(xnew):
            ynew = f(xnew)
            if ynew >= yold + backtrack_alpha * s * xstep.dot(gradx):
                return xnew
            if backtrack_iter >= max_backtrack_iter:
                # todo: return some status info saying that something bad happened
                return x
        s *= backtrack_beta
        backtrack_iter += 1
        
def linear_solve(A, b):
    # todo: block matrix stuff
    return np.linalg.solve(A,b)

#def test_gurobi():

def convert_gurobi_model(model):
    var2ind = {}
    
    nvar = model.getAttr("NumVars")
    ncnt = model.getAttr("NumConstrs")
    
    eqcnt = []
    ineqcnt = []
    for cnt in model.getConstrs(): 
        if cnt.getAttr("Sense")=="=": eqcnt.append(cnt)
        else: ineqcnt.append(cnt)
    nineqcnt = len(ineqcnt)
    neqcnt = len(eqcnt)
    
    H = np.zeros((nvar, nvar))
    f = np.zeros(nvar)
    A = np.zeros((neqcnt, nvar))
    b = np.zeros(neqcnt)
    C = np.zeros((nineqcnt, nvar))
    d = np.zeros(nineqcnt)
    ub = np.zeros(nvar)
    lb = np.zeros(nvar)
    def get_index(v):
        if v not in var2ind:
            var2ind[v] = len(var2ind)
        return var2ind[v]
    
    obj = model.getObjective()
    n_triples = obj.size()
    for i in xrange(n_triples):
        v1 = obj.getVar1(i)
        v2 = obj.getVar2(i)
        c = obj.getCoeff(i)
        H[get_index(v1), get_index(v2)] += c
        
    linexpr = obj.getLinExpr()
    n_lin = linexpr.size()
    for i in xrange(n_lin):
        v = linexpr.getVar(i)
        c = linexpr.getCoeff(i)
        f[get_index(v)] += c
        
    eqcntname = np.zeros(neqcnt,"|S12")
    for (i_row,cnt) in enumerate(eqcnt):
        linexpr = model.getRow(cnt)
        for i_var in xrange(linexpr.size()):
            A[i_row, get_index(linexpr.getVar(i_var))] = linexpr.getCoeff(i_var)
            b[i_row] = cnt.getAttr("RHS")
        eqcntname[i_row]
    ineqcntname = np.zeros(nineqcnt,"|S12")
    for (i_row,cnt) in enumerate(ineqcnt):
        linexpr = model.getRow(cnt)
        for i_var in xrange(linexpr.size()):
            C[i_row, get_index(linexpr.getVar(i_var))] = linexpr.getCoeff(i_var)
            d[i_row] = cnt.getAttr("RHS")
        ineqcntname[i_row] = cnt.getAttr("ConstrName")
    name = np.zeros(nvar, "|S12")
    for (i_var, var) in enumerate(model.getVars()):
        ub[i_var] = var.getAttr("UB")
        lb[i_var] = var.getAttr("LB")
        name[i_var] = var.getAttr("VarName")
    return H,f,A,b,C,d,lb, ub, name, eqcntname, ineqcntname
        
        
def test_solve():
    n_vars = 3
    H = np.eye(n_vars)
    f = np.ones(n_vars)
    C = np.ones((1,n_vars))
    d = np.ones(1) * 100
    lb = np.ones(n_vars)*0
    ub = np.ones(n_vars)*3
    #A, B, ub, lb, start
    xsol = solve_iqp(H, f, C, d, lb, ub,np.ones(n_vars))
    print xsol
if __name__ == "__main__":
    #test_gurobi()
#def test_convert():
    import gurobipy
    model = gurobipy.read("arm_table.mps")
    #model.optimize()
    H,f,A,b,C,d, lb, ub, varname, eqcntname, ineqcntname = convert_gurobi_model(model)
    
    xopt = solve_qp(H,f,A,b,C,d,lb, ub)
    
    model.optimize()
    vals = np.array([var.getAttr(gurobipy.GRB.attr.X) for var in model.getVars()])
    
    print vals[0*7:(4*7)] - xopt[1*7:5*7]