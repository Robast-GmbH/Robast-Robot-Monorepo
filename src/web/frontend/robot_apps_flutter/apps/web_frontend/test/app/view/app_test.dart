import 'package:flutter_test/flutter_test.dart';
import 'package:web_frontend/main.dart';
import 'package:web_frontend/pages/fleet_management_page.dart';

void main() {
  group('App', () {
    testWidgets('renders CounterPage', (tester) async {
      await tester.pumpWidget(const WebFrontend());
      expect(find.byType(FleetManagementPage), findsOneWidget);
    });
  });
}
