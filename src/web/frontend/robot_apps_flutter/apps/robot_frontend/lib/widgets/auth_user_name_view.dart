import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/user_provider.dart';

class AuthUserNameView extends StatelessWidget {
  const AuthUserNameView({super.key});

  @override
  Widget build(BuildContext context) {
    return FutureBuilder(
        future: Provider.of<UserProvider>(context, listen: false).getUserSession(robotName: 'rb_theron'),
        builder: (context, snapshot) {
          if (snapshot.connectionState == ConnectionState.waiting) {
            return const CircularProgressIndicator();
          }
          if (snapshot.hasError) {
            return const Text('Error');
          }
          if (snapshot.hasData) {
            final userGroups = snapshot.data!.userGroups.fold<String>('', (previousValue, element) => ' $element$previousValue');
            return Text('User: ${snapshot.data!.firstName} ${snapshot.data!.lastName} $userGroups');
          }
          return const Text('No Data');
        },);
  }
}
