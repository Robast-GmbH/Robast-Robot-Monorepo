import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_frontend/models/provider/robot_provider.dart';
import 'package:robot_frontend/views/home_view.dart';
import 'package:robot_frontend/widgets/background_view.dart';
import 'package:shared_preferences/shared_preferences.dart';

class ConfigView extends StatefulWidget {
  const ConfigView({super.key, this.autoClose = false});

  final bool autoClose;

  @override
  State<ConfigView> createState() => _ConfigViewState();
}

class _ConfigViewState extends State<ConfigView> {
  final _formKey = GlobalKey<FormState>();
  String robotBackendAddress = '';
  Future<SharedPreferences>? getSharedPreferencesFuture;

  Future<SharedPreferences> getSharedPreferences() async {
    return await SharedPreferences.getInstance();
  }

  Future<void> finishConfiguration({required String robotBackendAddress}) async {
    await Provider.of<RobotProvider>(context, listen: false).initRobotAPI(prefix: robotBackendAddress);
    await Navigator.pushReplacement(
      context,
      MaterialPageRoute(builder: (context) => HomeView()),
    );
  }

  @override
  void initState() {
    super.initState();
    getSharedPreferencesFuture = getSharedPreferences();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: BackgroundView(
        child: Row(
          children: [
            Expanded(child: SizedBox()),
            Expanded(
              child: FutureBuilder<SharedPreferences>(
                future: getSharedPreferencesFuture,
                builder: (context, snapshot) {
                  if (snapshot.connectionState == ConnectionState.done) {
                    if (snapshot.data!.getString('robotBackendAddress') != null && widget.autoClose) {
                      WidgetsBinding.instance.addPostFrameCallback((_) async {
                        await finishConfiguration(robotBackendAddress: robotBackendAddress);
                      });
                      return SizedBox();
                    }
                    return Form(
                      key: _formKey,
                      child: Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: <Widget>[
                          Padding(
                            padding: const EdgeInsets.all(8.0),
                            child: TextFormField(
                              initialValue: snapshot.data!.getString('robotBackendAddress') ?? '',
                              decoration: InputDecoration(
                                labelText: 'Enter the address of the robot backend',
                                hintText: 'http://ip:port',
                              ),
                              validator: (value) {
                                if (value == null || value.isEmpty) {
                                  return 'Please enter some text';
                                }
                                return null;
                              },
                              onSaved: (value) {
                                robotBackendAddress = value!;
                              },
                            ),
                          ),
                          Padding(
                            padding: const EdgeInsets.all(8.0),
                            child: ElevatedButton(
                              onPressed: () async {
                                if (_formKey.currentState!.validate()) {
                                  _formKey.currentState!.save();
                                  snapshot.data!.setString('robotBackendAddress', robotBackendAddress);
                                }
                                if (!widget.autoClose) {
                                  await finishConfiguration(robotBackendAddress: robotBackendAddress);
                                }
                              },
                              child: Padding(
                                padding: const EdgeInsets.symmetric(vertical: 8.0, horizontal: 32),
                                child: Text('Submit'),
                              ),
                            ),
                          ),
                        ],
                      ),
                    );
                  } else {
                    return Center(child: CircularProgressIndicator());
                  }
                },
              ),
            ),
            Expanded(child: SizedBox()),
          ],
        ),
      ),
    );
  }
}
