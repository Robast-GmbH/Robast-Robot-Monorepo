import 'package:flutter_test/flutter_test.dart';
import 'package:robot_frontend/main.dart';
import 'package:robot_frontend/views/home_view.dart';

void main() {
  group('App', () {
    testWidgets('renders HomeView', (tester) async {
      await tester.pumpWidget(const RobotFrontend());
      expect(find.byType(HomeView), findsOneWidget);
    });
  });
}
