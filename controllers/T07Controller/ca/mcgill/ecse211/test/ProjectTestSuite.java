package ca.mcgill.ecse211.test;

import org.junit.platform.runner.JUnitPlatform;
import org.junit.platform.suite.api.SelectPackages;
import org.junit.runner.RunWith;

/**
 * Project test suite for ECSE211 robotics project.
 * 
 * <p>A test suite groups multiple test files so they can be all run at once. To run this test
 * suite, right-click ProjectTestSuite.java and select Run as > JUnit test.
 * 
 * @author Younes Boubekeur
 */
@RunWith(JUnitPlatform.class)
@SelectPackages("ca.mcgill.ecse211.test")
public class ProjectTestSuite {
  // the class body remains empty, used only as a holder for the above annotations
}