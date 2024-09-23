import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/models/provider/fleet_provider.dart';
import 'package:web_frontend/models/provider/map_provider.dart';
import 'package:web_frontend/models/provider/task_provider.dart';
import 'package:web_frontend/models/provider/user_provider.dart';

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
    Provider.of<FleetProvider>(context, listen: false).initMiddlewarAPI(prefix: middlewareAddress);
    Provider.of<TaskProvider>(context, listen: false).initMiddlewarAPI(prefix: middlewareAddress);
    Provider.of<MapProvider>(context, listen: false).initMiddlewarAPI(prefix: middlewareAddress);
    Provider.of<UserProvider>(context, listen: false).initMiddlewarAPI(prefix: middlewareAddress);

    Navigator.pop(
      context,
    );
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
        title: Row(
          children: [
            Icon(Icons.settings),
            SizedBox(width: 8),
            const Text('Netzwerkeinstellungen'),
          ],
        ),
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
                      children: <Widget>[
                        Padding(
                          padding: const EdgeInsets.all(16),
                          child: TextFormField(
                            style: TextStyle(color: WebColors.secondaryText),
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
                                await finishConfiguration();
                              }
                            },
                            child: const Padding(
                              padding: EdgeInsets.symmetric(vertical: 8, horizontal: 32),
                              child: Text('Bestätigen'),
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
