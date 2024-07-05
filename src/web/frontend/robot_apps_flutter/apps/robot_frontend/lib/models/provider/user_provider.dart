import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';

class UserProvider extends ChangeNotifier {
  final availableTitles = ['', 'Dr.', 'Prof.', 'Prof. Dr.', 'PD Dr.', 'PD Dr. Dr.'];

  late MiddlewareApiUtilities _middlewareApiUtilities;

  void initMiddlewareApiUtilities({required String prefix}) {
    _middlewareApiUtilities = MiddlewareApiUtilities(prefix: prefix);
  }

  Future<List<User>> getUsers() async {
    return _middlewareApiUtilities.users.getUsers();
  }

  Future<User?> getUser({required String id}) async {
    return _middlewareApiUtilities.users.getUser(id: id);
  }

  Future<User?> updateUser({required User updatedUser}) async {
    return _middlewareApiUtilities.users.updateUser(updatedUser: updatedUser);
  }

  Future<User?> createUser({required User newUser}) async {
    return _middlewareApiUtilities.users.createUser(newUser: newUser);
  }

  Future<bool> deleteUser({required String id}) async {
    return _middlewareApiUtilities.users.deleteUser(id: id);
  }
}
