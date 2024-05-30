// ignore_for_file: prefer_const_constructors
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:test/test.dart';

void main() {
  group('MiddlewareApiUtilities', () {
    test('can be instantiated', () {
      expect(MiddlewareApiUtilities(prefix: ''), isNotNull);
    });
  });
}
