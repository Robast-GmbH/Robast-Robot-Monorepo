import 'package:flutter_test/flutter_test.dart';
import 'package:web_frontend/main.dart';
import 'package:web_frontend/views/fleet_management_view.dart';

void main() {
  group('App', () {
    testWidgets('renders CounterPage', (tester) async {
      await tester.pumpWidget(const WebFrontend());
      expect(find.byType(FleetManagementView), findsOneWidget);
    });
  });
}
