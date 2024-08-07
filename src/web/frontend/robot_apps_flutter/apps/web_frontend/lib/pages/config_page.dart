import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';
import 'package:web_frontend/pages/fleet_management_page.dart';

class ConfigPage extends StatefulWidget {
  const ConfigPage({
    super.key,
    this.autoClose = false,
  });

  final bool autoClose;

  @override
  State<ConfigPage> createState() => _ConfigPageState();
}

class _ConfigPageState extends State<ConfigPage> {
  final _formKey = GlobalKey<FormState>();
  String middlewareAddress = '';
  late Future<SharedPreferences> loadSharedPreferences;

  Future<void> finishConfiguration() async {
    await Provider.of<FleetProvider>(context, listen: false).initMiddlewarAPI(prefix: middlewareAddress);
    if (mounted) {
      await Navigator.pushReplacement(
        context,
        MaterialPageRoute<FleetManagementPage>(
          builder: (context) => const FleetManagementPage(),
        ),
      );
    }
  }

  @override
  void initState() {
    super.initState();
    loadSharedPreferences = SharedPreferences.getInstance();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Settings'),
      ),
      body: Row(
        children: [
          Expanded(
            child: FutureBuilder<SharedPreferences>(
              future: loadSharedPreferences,
              builder: (context, snapshot) {
                if (snapshot.connectionState == ConnectionState.done) {
                  final sharedPreferences = snapshot.data!;
                  if (sharedPreferences.getString('middlewareAddress') != null && widget.autoClose) {
                    WidgetsBinding.instance.addPostFrameCallback((_) async {
                      middlewareAddress = sharedPreferences.getString('middlewareAddress') ?? '';
                      await finishConfiguration();
                    });
                    return const SizedBox();
                  }
                  return Form(
                    key: _formKey,
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: <Widget>[
                        Padding(
                          padding: const EdgeInsets.all(8),
                          child: TextFormField(
                            initialValue: sharedPreferences.getString('middlewareAddress') ?? '',
                            decoration: const InputDecoration(
                              labelText: 'Middleware IP',
                              border: OutlineInputBorder(),
                              prefixIcon: Icon(Icons.wifi),
                            ),
                            keyboardType: const TextInputType.numberWithOptions(decimal: true),
                            validator: (value) {
                              final ipPattern = RegExp(r'^http://\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}:\d{1,5}$');
                              if (!ipPattern.hasMatch(value!)) {
                                return 'Please enter a valid URL in the format http://ip:port';
                              }
                              return null;
                            },
                            onSaved: (value) {
                              middlewareAddress = value!;
                            },
                          ),
                        ),
                        Padding(
                          padding: const EdgeInsets.all(8),
                          child: ElevatedButton(
                            onPressed: () async {
                              if (_formKey.currentState!.validate()) {
                                _formKey.currentState!.save();
                                await snapshot.data!.setString('middlewareAddress', middlewareAddress);
                              }
                              if (!widget.autoClose) {
                                await finishConfiguration();
                              }
                            },
                            child: const Padding(
                              padding: EdgeInsets.symmetric(vertical: 8, horizontal: 32),
                              child: Text('Submit'),
                            ),
                          ),
                        ),
                      ],
                    ),
                  );
                } else {
                  return const Center(child: CircularProgressIndicator());
                }
              },
            ),
          ),
        ],
      ),
    );
  }
}
