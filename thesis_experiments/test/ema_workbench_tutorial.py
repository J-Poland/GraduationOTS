from ema_workbench import Model, RealParameter, ScalarOutcome, ema_logging, perform_experiments


# simple python model
def some_model(x1=None, x2=None, x3=None):
    return {'y': x1*x2+x3}


# setup EMA Workbench experiments
if __name__ == '__main__':
    ema_logging.LOG_FORMAT = '[%(name)s/%(levelname)s/%(processName)s] %(message)s'
    ema_logging.log_to_stderr(ema_logging.INFO)

    # instantiate model object
    model = Model('simpleModel', function=some_model)

    # specify uncertainties
    model.uncertainties = [RealParameter("x1", 0.1, 10),
                           RealParameter("x2", -0.01,0.01),
                           RealParameter("x3", -0.01,0.01)]

    # specify outcomes
    model.outcomes = [ScalarOutcome('y')]

    # run experiments
    iterations = 100
    results = perform_experiments(model, iterations)

    # show last x results
    print(results[1])
