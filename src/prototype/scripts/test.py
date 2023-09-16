import os
import pprint


env_var=os.environ

print("User's Environment variable:")
pprint.pprint(dict(env_var), width=1)