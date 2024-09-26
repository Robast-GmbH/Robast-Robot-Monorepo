import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_frontend/models/provider/user_provider.dart';
import 'package:web_frontend/pages/change_password_page.dart';
import 'package:web_frontend/pages/home_page.dart';
import 'package:web_frontend/validators.dart';
import 'package:web_frontend/widgets/custom_text_field.dart';

class LoginPage extends StatelessWidget {
  LoginPage({super.key});
  final formKey = GlobalKey<FormState>();
  final mailController = TextEditingController(text: 'ane@robast.de');
  final passwordController = TextEditingController(text: 'Robast24!');

  @override
  Widget build(BuildContext context) {
    var isLoading = false;
    var showLoginFailed = false;
    return Scaffold(
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Form(
          key: formKey,
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Image.asset(
                    'assets/robast_logo.png',
                    height: 64,
                  ),
                  const SizedBox(
                    width: 16,
                  ),
                  const Text(
                    'Robast',
                    style: TextStyle(fontSize: 48, color: Colors.white),
                  ),
                ],
              ),
              const SizedBox(height: 32),
              CustomTextField(
                controller: mailController,
                label: 'E-Mail',
                validator: Validators.mailValidator,
                prefixIcon: const Icon(Icons.mail_outline),
              ),
              const SizedBox(height: 16),
              CustomTextField(
                controller: passwordController,
                label: 'Passwort',
                validator: (value) {
                  if (value == null || value.isEmpty) {
                    return 'Bitte geben Sie ihr Passwort ein';
                  }
                  return null;
                },
                prefixIcon: const Icon(Icons.lock_outline),
                obsucureText: true,
              ),
              const SizedBox(height: 16),
              StatefulBuilder(
                builder: (context, setState) {
                  return ElevatedButton.icon(
                    onPressed: () async {
                      if (formKey.currentState!.validate()) {
                        isLoading = true;
                        setState(() {});
                        final wasSuccessful = await Provider.of<UserProvider>(context, listen: false).login(
                          mail: mailController.text,
                          password: passwordController.text,
                        );
                        isLoading = false;
                        setState(() {});
                        if (wasSuccessful && context.mounted) {
                          await Navigator.of(context).pushReplacement(
                            Validators.passwordValidator(passwordController.text) == null
                                ? MaterialPageRoute<HomePage>(
                                    builder: (context) => const HomePage(),
                                  )
                                : MaterialPageRoute<ChangePasswordPage>(
                                    builder: (context) => ChangePasswordPage(
                                      forceChange: true,
                                      initialInput: passwordController.text,
                                    ),
                                  ),
                          );
                        } else {
                          showLoginFailed = !wasSuccessful;
                          setState(() {});
                          Future.delayed(const Duration(seconds: 1), () {
                            showLoginFailed = false;
                            setState(() {});
                          });
                        }
                      }
                    },
                    label: const Padding(
                      padding: EdgeInsets.symmetric(vertical: 16),
                      child: Text(
                        'Anmelden',
                      ),
                    ),
                    icon: isLoading
                        ? const SizedBox.square(dimension: 24, child: CircularProgressIndicator())
                        : showLoginFailed
                            ? const Icon(Icons.clear)
                            : const Icon(Icons.login),
                  );
                },
              ),
            ],
          ),
        ),
      ),
    );
  }
}
