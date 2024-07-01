import 'package:flutter/material.dart';
import 'package:robot_frontend/widgets/custom_scaffold.dart';

class UserManagementPage extends StatefulWidget {
  const UserManagementPage({super.key});

  @override
  State<UserManagementPage> createState() => _UserManagementPageState();
}

class _UserManagementPageState extends State<UserManagementPage> {
  @override
  Widget build(BuildContext context) {
    return const CustomScaffold(
      title: 'User Management',
    );
  }
}
