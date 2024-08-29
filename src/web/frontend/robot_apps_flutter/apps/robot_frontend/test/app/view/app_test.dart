import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:robot_frontend/main.dart';
import 'package:robot_frontend/pages/home_page.dart';

void main() {
  group('App', () {
    testWidgets('renders HomeView', (tester) async {
      final navigatorKey = GlobalKey<NavigatorState>();
      await tester.pumpWidget(RobotFrontend(
        navigatorKey: navigatorKey,
      ));
      expect(find.byType(HomePage), findsOneWidget);
    });
  });
}
