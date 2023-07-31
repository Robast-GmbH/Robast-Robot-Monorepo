import 'package:flutter/material.dart';
import 'package:web_interface_flutter/pages/map_page.dart';

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
                  hintText: 'What do people call you?',
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
                  hintText: 'What is your secret?',
                  labelText: 'Password',
                ),
                controller: passwordController,
                obscureText: true,
              ),
            ),
            const SizedBox(
              height: 16,
            ),
            Padding(
              padding: const EdgeInsets.all(8.0),
              child: ElevatedButton.icon(
                  onPressed: () {
                    print(nameController.text);
                    print(passwordController.text);
                    Navigator.pushReplacement(context, MaterialPageRoute(builder: (context) => const MapPage()));
                  },
                  icon: const Icon(Icons.login),
                  label: const Text("Login")),
            )
          ],
        ),
      ),
    );
  }
}
