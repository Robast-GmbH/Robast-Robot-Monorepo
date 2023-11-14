import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_interface_flutter/constants/constants.dart';
import 'package:web_interface_flutter/constants/gaps.dart';
import 'package:web_interface_flutter/models/provider/robot_provider.dart';
import 'package:web_interface_flutter/pages/home_page.dart';

class LoginPage extends StatefulWidget {
  const LoginPage({super.key});

  @override
  State<LoginPage> createState() => _LoginPageState();
}

class _LoginPageState extends State<LoginPage> {
  final nameController = TextEditingController();
  final passwordController = TextEditingController();

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            SizedBox(
              width: 300,
              child: TextFormField(
                decoration: const InputDecoration(
                  icon: Icon(Icons.person),
                  labelText: 'Name',
                ),
                controller: nameController,
              ),
            ),
            SizedBox(
              width: 300,
              child: TextFormField(
                decoration: const InputDecoration(
                  icon: Icon(Icons.lock),
                  labelText: 'Passwort',
                ),
                controller: passwordController,
                obscureText: true,
              ),
            ),
            Gaps.mediumVertical,
            Padding(
              padding: Constants.smallPadding,
              child: TextButton.icon(
                onPressed: () {
                  if (nameController.text == "admin" && passwordController.text == "RobastGmbH2021") {
                    Provider.of<RobotProvider>(context, listen: false).isAdmin = true;
                  }
                  Navigator.pushReplacement(
                    context,
                    MaterialPageRoute(
                      builder: (context) => const HomePage(),
                    ),
                  );
                },
                icon: const Icon(Icons.login),
                label: const Text("Login"),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
