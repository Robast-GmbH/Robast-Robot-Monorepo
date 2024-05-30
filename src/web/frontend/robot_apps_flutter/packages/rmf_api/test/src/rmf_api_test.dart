// ignore_for_file: prefer_const_constructors
import 'package:rmf_api/rmf_api.dart';
import 'package:test/test.dart';

void main() {
  group('RmfApi', () {
    const prefix = 'http://localhost:8000';
    test('can be instantiated', () {
      expect(RmfApi(prefix: prefix), isNotNull);
    });

    test('getBuildingMap runs successfully', () async {
      try {
        final rmfApi = RmfApi(prefix: prefix);
        final result = await rmfApi.getBuildingMap();
        expect(result, isNotNull);
      } catch (e) {
        fail('getBuildingMap threw an exception: $e');
      }
    });

    test('getTasks runs successfully', () async {
      try {
        final rmfApi = RmfApi(prefix: prefix);
        final tasks = await rmfApi.getTasks();
        expect(tasks, isNotNull);
      } catch (e) {
        fail('dispatchTask threw an exception: $e');
      }
    });

    test('dispatchPatrolTask runs successfully', () async {
      try {
        final rmfApi = RmfApi(prefix: prefix);
        await rmfApi.dispatchPatrolTask(places: ['storage', 'tt'], rounds: 2);
      } catch (e) {
        fail('dispatchTask threw an exception: $e');
      }
    });
  });
}
