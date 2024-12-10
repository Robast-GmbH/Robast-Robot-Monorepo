// ignore_for_file: prefer_const_constructors
import 'package:robot_api_utilities/robot_api_utilities.dart';
import 'package:test/test.dart';

void main() {
  group('RobotApiUtilities', () {
    const testPrefix = 'http://10.10.23.8:8001';

    test('can be instantiated', () {
      expect(RobotApiUtilities(prefix: ''), isNotNull);
    });

    test('can get is_navigation_blocked', () async {
      final robotApiUtilities = RobotApiUtilities(prefix: testPrefix);
      final isNavigationBlocked = await robotApiUtilities.isNavigationBlocked();
      expect(isNavigationBlocked, isNotNull);
    });

    test('can get hearbeat_timeout_errors', () async {
      final robotApiUtilities = RobotApiUtilities(prefix: testPrefix);
      final hearbeatTimeoutErrors = await robotApiUtilities.getHeartbeatTimeoutErrors();
      expect(hearbeatTimeoutErrors, isNotNull);
    });

    test('can get living devices', () async {
      final robotApiUtilities = RobotApiUtilities(prefix: testPrefix);
      final livingDevices = await robotApiUtilities.getLivingDevices();
      expect(livingDevices, isNotNull);
    });
  });
}
