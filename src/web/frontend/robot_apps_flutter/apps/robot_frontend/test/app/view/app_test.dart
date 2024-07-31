import 'package:flutter_test/flutter_test.dart';
import 'package:robot_frontend/main.dart';
import 'package:robot_frontend/pages/home_page.dart';

void main() {
  group('App', () {
    testWidgets('renders HomeView', (tester) async {
      await tester.pumpWidget(const RobotFrontend());
      expect(find.byType(HomePage), findsOneWidget);
    });
  });
}
