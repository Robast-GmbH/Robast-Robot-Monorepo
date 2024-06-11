// ignore_for_file: prefer_const_constructors
import 'package:robot_api_utilities/robot_api_utilities.dart';
import 'package:test/test.dart';

void main() {
  group('RobotApiUtilities', () {
    final testPrefix = 'http://10.10.23.7:8001';

    test('can be instantiated', () {
      expect(RobotApiUtilities(prefix: ''), isNotNull);
    });

    test('can get is_navigation_blocked', () async {
      final robotApiUtilities = RobotApiUtilities(prefix: testPrefix);
      final isNavigationBlocked = await robotApiUtilities.isNavigationBlocked();
      expect(isNavigationBlocked, isNotNull);
    });
    test('can get module_process_status', () async {
      final robotApiUtilities = RobotApiUtilities(prefix: testPrefix);
      final moduleProcess = await robotApiUtilities.getModuleProcess();
      expect(moduleProcess, isNotNull);
    });
  });
}
