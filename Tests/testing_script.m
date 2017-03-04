%Script that runs through the 

%--- Clean and Clear ---%
clc
clear
close all

%Configure code coverage report
import matlab.unittest.TestRunner
import matlab.unittest.plugins.CodeCoveragePlugin
runner = TestRunner.withTextOutput;
runner.addPlugin(CodeCoveragePlugin.forFolder('../KinDyn'))

%Define test suite
suite = testsuite('test_SPART');

%Run tests
result = runner.run(suite);