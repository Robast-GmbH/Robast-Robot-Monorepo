from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
import yaml

# config = yaml.safe_load(open("./src/config.yml"))
# POSTGRES_USER : str = config['POSTGRES_USER']
# POSTGRES_PASSWORD = config['POSTGRES_PASSWORD']
# POSTGRES_SERVER : str = config['POSTGRES_SERVER']#"localhost")
# POSTGRES_PORT : str = config['POSTGRES_PORT']#5432) # default postgres port is 5432
# POSTGRES_DB : str = config['POSTGRES_DB']#,"tdd")
# DATABASE_URL = f"postgresql://{POSTGRES_USER}:{POSTGRES_PASSWORD}@{POSTGRES_SERVER}:{POSTGRES_PORT}/{POSTGRES_DB}"
#SQLALCHEMY_DATABASE_URL = "postgresql://postgres:postgres@localhost:5432/robast"

SQLALCHEMY_DATABASE_URL = "sqlite:///./sql_app.db"
engine = create_engine( SQLALCHEMY_DATABASE_URL, connect_args={"check_same_thread": False})
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()